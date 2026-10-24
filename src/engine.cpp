#include "engine.h"

#include <algorithm>
#include <ctime>
#include <fstream>
#include <stdexcept>

#include "dsp.h"
#include "itc.h"
#include "parse.h"

namespace openitc {
namespace {

struct Found {
    int type;
    std::vector<uint8_t> payload;
    int errors;
};

// Demod + FEC-decode one channel's baseband; returns unique packets and adds
// the sync-hit count to `sync_hits`.
std::vector<Found> decode_stream(const std::vector<cf>& y, int max_errors,
                                 unsigned long long& sync_hits) {
    const int sps = 12;
    std::vector<Found> out;
    std::vector<std::string> seen;
    for (int o = 0; o < sps; ++o) {
        std::vector<uint8_t> bits;
        pi4dqpsk_demod(y, sps, o, bits);
        auto hits = find_sync(bits, max_errors);
        sync_hits += hits.size();
        for (auto h : hits) {
            auto bytes = pack_bits_msb(bits, h.bit_pos, kUniqueWordLen + 6 * 8 + 256 * 8);
            if (bytes.size() < 14) continue;
            ItcPacket pkt;
            if (!decode_packet(bytes, pkt) || !pkt.valid) continue;
            std::string key((const char*)pkt.payload.data(), pkt.payload.size());
            key.push_back((char)pkt.type);
            if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
            seen.push_back(key);
            out.push_back({pkt.type, pkt.payload, h.errors});
        }
    }
    return out;
}

}  // namespace

MonitorEngine::MonitorEngine(EngineConfig cfg) : cfg_(std::move(cfg)) {}

MonitorEngine::~MonitorEngine() { stop(); }

std::string MonitorEngine::device_name() const {
    return rtl_device_name(cfg_.rtl.device_index);
}

std::string MonitorEngine::last_error() const {
    std::lock_guard<std::mutex> lk(mu_);
    return last_error_;
}

EngineStats MonitorEngine::stats() const {
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}

void MonitorEngine::emit(const PacketEvent& e) {
    if (cb_) cb_(e);
}

bool MonitorEngine::start() {
    if (running_.load()) return false;
    try {
        if (!cfg_.iq_file.empty()) {
            std::ifstream test(cfg_.iq_file, std::ios::binary | std::ios::ate);
            if (!test) throw std::runtime_error("cannot open IQ file: " + cfg_.iq_file);
            const auto bytes = test.tellg();
            if (bytes < 0 || bytes % 2 != 0)
                throw std::runtime_error("IQ file has an odd byte count");
        } else {
            src_ = std::make_unique<RtlSource>(cfg_.rtl);
            src_->open_device();
            src_->configure();
        }
    } catch (const std::exception& ex) {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = ex.what();
        src_.reset();
        return false;
    }
    running_.store(true);
    th_ = std::thread([this] { run(); });
    return true;
}

void MonitorEngine::stop() {
    running_.store(false);
    if (th_.joinable()) th_.join();
    src_.reset();
}

void MonitorEngine::run() {
    const uint32_t fs = cfg_.rtl.sample_rate;
    if (fs == 0 || fs % 192000u != 0u) {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = "sample rate must be a multiple of 192000";
        running_.store(false);
        return;
    }
    const int fr_factor = (int)(fs / 192000u);
    FirDecimator decfr(fr_factor, 0.45 * 192000.0 / fs, 16 * fr_factor + 1);
    FirDecimator dechr(2, 0.45 * 96000.0 / 192000.0, 33);

    const size_t chunk = (size_t)fs * (size_t)cfg_.chunk_ms / 1000;
    unsigned long long seq = 0;
    unsigned long long samples_processed = 0;
    std::ifstream file;
    if (!cfg_.iq_file.empty()) {
        file.open(cfg_.iq_file, std::ios::binary);
        if (!file) {
            std::lock_guard<std::mutex> lk(mu_);
            last_error_ = "cannot open IQ file: " + cfg_.iq_file;
            running_.store(false);
            return;
        }
    }
    std::vector<ChannelStats> totals;
    for (int ch : cfg_.channels) {
        if (cfg_.decode_fr) totals.push_back({ch, 0, 0, 0});
        if (cfg_.decode_hr) totals.push_back({ch, 1, 0, 0});
    }

    while (running_.load()) {
        std::vector<uint8_t> iq;
        try {
            if (file.is_open()) {
                iq.resize(chunk * 2);
                file.read(reinterpret_cast<char*>(iq.data()),
                          static_cast<std::streamsize>(iq.size()));
                const auto nbytes = file.gcount();
                if (nbytes == 0) {
                    if (file.bad()) throw std::runtime_error("IQ file read error");
                    break;  // normal EOF
                }
                if (nbytes % 2 != 0)
                    throw std::runtime_error("partial I/Q pair at end of file");
                iq.resize(static_cast<size_t>(nbytes));
            } else {
                src_->read(iq, chunk);
            }
        } catch (const std::exception& ex) {
            std::lock_guard<std::mutex> lk(mu_);
            last_error_ = ex.what();
            break;
        }
        const double capture_offset = double(samples_processed) / fs;
        samples_processed += iq.size() / 2;
        std::vector<cf> x0 = to_baseband(iq);

        unsigned long long local_hits = 0;
        std::vector<PacketEvent> events;
        for (int ch : cfg_.channels) {
            std::vector<cf> x = x0;
            mix_shift(x, (double)itc_frequency((uint32_t)ch) - cfg_.rtl.center_freq, fs);
            std::vector<cf> yfr;
            decfr.process(x, yfr);
            std::vector<cf> yhr;
            dechr.process(yfr, yhr);
            const std::vector<cf>* streams[2] = {&yfr, &yhr};
            for (int m = 0; m < 2; ++m) {
                if (m == 0 && !cfg_.decode_fr) continue;
                if (m == 1 && !cfg_.decode_hr) continue;
                const auto before = local_hits;
                auto found = decode_stream(*streams[m], cfg_.max_errors, local_hits);
                for (auto& st : totals) {
                    if (st.channel == ch && st.mode == m) {
                        st.sync_hits += local_hits - before;
                        st.packets += found.size();
                        break;
                    }
                }
                for (auto& f : found) {
                    PacketEvent e;
                    e.seq = ++seq;
                    e.t_unix = file.is_open()
                        ? (cfg_.capture_start_unix > 0
                            ? cfg_.capture_start_unix + capture_offset
                            : -1.0 - capture_offset)
                        : (double)std::time(nullptr);
                    e.channel = ch;
                    e.mode = m;
                    e.type = f.type;
                    e.sync_errors = f.errors;
                    e.payload = std::move(f.payload);
                    parse_event(e);
                    events.push_back(std::move(e));
                }
            }
        }

        {
            std::lock_guard<std::mutex> lk(mu_);
            stats_.chunks++;
            stats_.sync_hits += local_hits;
            stats_.packets += events.size();
            stats_.channels = totals;
        }
        for (const auto& e : events) emit(e);
    }
    running_.store(false);
}

}  // namespace openitc
