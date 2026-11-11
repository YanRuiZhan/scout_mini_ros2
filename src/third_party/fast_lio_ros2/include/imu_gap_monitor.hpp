#ifndef FAST_LIO_IMU_GAP_MONITOR_HPP
#define FAST_LIO_IMU_GAP_MONITOR_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>

// Detects missing IMU samples from header stamps alone.
//
// FAST-LIO integrates every IMU sample between two scans. A sample lost before
// it reaches the node (bridge queue overflow, best-effort transport) is
// invisible afterwards: the integration just bridges the hole with one long
// step. This keeps a running estimate of the nominal sample period and reports
// every interval that is clearly longer.
class ImuGapMonitor
{
public:
    explicit ImuGapMonitor(double gap_ratio = 1.5) : gap_ratio_(gap_ratio) {}

    // Feed the stamp (seconds) of the next sample. Returns the interval to the
    // previous sample when it is a gap, otherwise 0.
    double update(double stamp)
    {
        if (!has_last_ || stamp <= last_stamp_)
        {
            // First sample, duplicate, or a clock reset: restart the interval.
            has_last_ = true;
            last_stamp_ = stamp;
            return 0.0;
        }
        const double dt = stamp - last_stamp_;
        last_stamp_ = stamp;
        if (nominal_ <= 0.0)
        {
            nominal_ = dt;
            return 0.0;
        }
        if (dt > gap_ratio_ * nominal_)
        {
            if (++consecutive_gaps_ >= RELEARN_AFTER)
            {
                // Every interval looks like a gap: the estimate started from an
                // unusually short interval. Adopt the current one instead.
                nominal_ = dt;
                consecutive_gaps_ = 0;
                return 0.0;
            }
            gaps_++;
            lost_ += static_cast<std::size_t>(std::llround(dt / nominal_)) - 1;
            max_gap_ = std::max(max_gap_, dt);
            return dt;
        }
        consecutive_gaps_ = 0;
        // Track slow rate changes; shrink quickly so an estimate that started
        // from a gap recovers within a few samples.
        nominal_ += (dt < nominal_ ? 0.2 : 0.02) * (dt - nominal_);
        return 0.0;
    }

    double nominal_period() const { return nominal_; }
    std::size_t gaps() const { return gaps_; }
    std::size_t lost_samples() const { return lost_; }
    double max_gap() const { return max_gap_; }

private:
    static constexpr int RELEARN_AFTER = 5;
    double gap_ratio_;
    int consecutive_gaps_ = 0;
    bool has_last_ = false;
    double last_stamp_ = 0.0;
    double nominal_ = 0.0;
    std::size_t gaps_ = 0;
    std::size_t lost_ = 0;
    double max_gap_ = 0.0;
};

#endif
