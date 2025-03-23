#include <vector>
#include <cstdint>
#include <memory>
#include <unordered_map>

struct Frame
{
    std::vector<uint8_t> data;
    uint8_t type;
};

struct HeaderFooter
{
    std::vector<uint8_t> header;
    std::vector<uint8_t> footer;
};

enum class ProcessorState
{
    WaitingForHeader,
    ReadingData,
    WaitingForFooter,
};

class SensorProcessor
{
public:
    SensorProcessor(const std::unordered_map<uint8_t, HeaderFooter> &headerFooterMap) : headerFooterMap_(headerFooterMap) {}

    std::unique_ptr<Frame> processByte(uint8_t byte);
    ProcessorState getState() const { return state_; }
    void reset();

private:
    ProcessorState state_ = ProcessorState::WaitingForHeader;
    std::vector<uint8_t> buffer_;
    uint8_t bufferSize_ = 0;
    uint8_t footerStartPos_ = 0;
    uint8_t currentType_ = 0;

    const std::unordered_map<uint8_t, HeaderFooter> &headerFooterMap_;
};

std::unique_ptr<Frame> SensorProcessor::processByte(uint8_t byte)
{
    buffer_.push_back(byte);
    bufferSize_ += 1;
    switch (state_)
    {
    case ProcessorState::WaitingForHeader:
    {
        for (const auto &[type, headerFooter] : headerFooterMap_)
        {
            const auto &header = headerFooter.header;
            if (buffer_.size() == header.size() && std::equal(header.begin(), header.end(), buffer_.begin(), buffer_.end()))
            {
                currentType_ = type;
                state_ = ProcessorState::ReadingData;
                break;
            }
        }
        break;
    }
    case ProcessorState::ReadingData:
    {
        const auto &footer = headerFooterMap_.at(currentType_).footer;
        if (byte == footer[0])
        {
            footerStartPos_ = bufferSize_ - 1;
            state_ = ProcessorState::WaitingForFooter;
        }
        break;
    }
    case ProcessorState::WaitingForFooter:
    {
        const auto &footer = headerFooterMap_.at(currentType_).footer;
        if (bufferSize_ - footerStartPos_ == footer.size() && std::equal(footer.begin(), footer.end(), buffer_.begin() + footerStartPos_, buffer_.end()))
        {
            Frame frame{buffer_, currentType_};
            buffer_.clear();
            state_ = ProcessorState::WaitingForHeader;
            return std::make_unique<Frame>(frame);
        }
        else
        {
            state_ = ProcessorState::ReadingData;
        }
        break;
    }
    }
    return std::nullopt;
}

void SensorProcessor::reset()
{
    state_ = ProcessorState::WaitingForHeader;
    buffer_.clear();
    bufferSize_ = 0;
    footerStartPos_ = 0;
    currentType_ = 0;
}