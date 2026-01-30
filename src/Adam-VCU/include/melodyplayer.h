#pragma once

#include <cstdint>
#include <optional>

class MelodyPlayer
{
public:
    static constexpr uint8_t kIdleWarn[10] = {10, 0, 0, 0, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kCtrlWarn[10] = {2, 2, 0, 0, 2, 2, 0, 0, 2, 2};
    static constexpr uint8_t kFail[10] = {15, 15, 0, 1, 1, 1, 1, 1, 1, 1};
    static constexpr uint8_t kSuccess[10] = {1, 1, 0, 0, 5, 5, 5, 0, 0, 0};

    bool isPlaying() const;
    void start(const uint8_t *newTones);
    std::optional<uint8_t> tick();

private:
    const uint8_t *tones_ = nullptr;
    uint8_t index_ = 10;
    uint8_t slowdown_ = 0;
};