#include "melodyplayer.h"

bool MelodyPlayer::isPlaying() const
{
    return tones_ && index_ < 10;
}

void MelodyPlayer::start(const uint8_t *newTones)
{
    if (!isPlaying())
    {
        tones_ = newTones;
        index_ = 0;
        slowdown_ = 0;
    }
}

std::optional<uint8_t> MelodyPlayer::tick()
{
    if (!isPlaying())
    {
        return std::nullopt;
    }

    uint8_t val = tones_[index_];

    if (++slowdown_ == 3)
    {
        slowdown_ = 0;
        ++index_;
    }

    if (val == 0) {
        return std::nullopt; // don't play anything, state machine was advanced nevertheless
    }

    return val;
}