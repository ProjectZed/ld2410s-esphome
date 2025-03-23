#include <vector>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace std {
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}

struct Frame
{
    std::vector<uint8_t> data;
    uint8_t type;
};

struct HeaderDataFooter
{
    std::vector<uint8_t> header;
    size_t minDataSize;
    size_t maxDataSize;
    std::vector<uint8_t> footer;
};

enum class ProcessorState
{
    WaitingForHeaderStart,
    WaitingForHeaderEnd,
    WaitingForData,
    WaitingForFooterStart,
    WaitingForFooterEnd,
};

class SensorProcessor
{
public:
    SensorProcessor(const std::unordered_map<uint8_t, HeaderDataFooter> &headerDataFooterMap)
        : headerDataFooterMap_(headerDataFooterMap) {}

    std::unique_ptr<Frame> processByte(uint8_t byte)
    {
        if (state_ == ProcessorState::WaitingForHeaderStart)
        {
            for (const auto &pair : headerDataFooterMap_)
            {
                const auto &type = pair.first;
                const auto &headerFooter = pair.second;
                const auto &header = headerFooter.header;
                const auto &minDataSize = headerDataFooterMap_.at(currentType_).minDataSize;
                if (byte == header[0])
                {
                    buffer_.push_back(byte);
                    currentType_ = type;
                    if (header.size() == 1)
                    {
                        if (minDataSize == 0)
                        {
                            state_ = ProcessorState::WaitingForFooterStart;
                        } else {
                            dataStartpos_ = buffer_.size();
                            state_ = ProcessorState::WaitingForData;
                        }
                    }
                    else
                    {
                        state_ = ProcessorState::WaitingForHeaderEnd;
                    }
                    return nullptr;
                }
            }
            reset();
            return nullptr;
        }
        else if (state_ == ProcessorState::WaitingForHeaderEnd)
        {
            const auto &header = headerDataFooterMap_.at(currentType_).header;
            const auto &minDataSize = headerDataFooterMap_.at(currentType_).minDataSize;
            if (byte == header[buffer_.size()])
            {
                buffer_.push_back(byte);
                if (buffer_.size() == header.size())
                {
                    if (minDataSize == 0)
                    {
                        state_ = ProcessorState::WaitingForFooterStart;
                    }
                    else
                    {
                        state_ = ProcessorState::WaitingForData;
                        dataStartpos_ = buffer_.size();
                    }
                }
                return nullptr;
            }
            reset();
            return nullptr;
        }
        else if (state_ == ProcessorState::WaitingForData)
        {
            buffer_.push_back(byte);
            const auto &maxDataSize = headerDataFooterMap_.at(currentType_).maxDataSize;
            if (buffer_.size() - dataStartpos_ == maxDataSize)
            {
                state_ = ProcessorState::WaitingForFooterStart;
            }
            return nullptr;
        }
        else if (state_ == ProcessorState::WaitingForFooterStart)
        {
            const auto &footer = headerDataFooterMap_.at(currentType_).footer;
            if (byte == footer[0])
            {
                footerStartPos_ = buffer_.size();
                buffer_.push_back(byte);
                if (footer.size() == 1)
                {
                    Frame frame{buffer_, currentType_};
                    reset();
                    return std::make_unique<Frame>(frame);
                }
                else
                {
                    state_ = ProcessorState::WaitingForFooterEnd;
                }
            }
            else
            {
                buffer_.push_back(byte);
            }
            return nullptr;
        }
        else if (state_ == ProcessorState::WaitingForFooterEnd)
        {
            const auto &footer = headerDataFooterMap_.at(currentType_).footer;
            if (byte == footer[buffer_.size() - footerStartPos_])
            {
                buffer_.push_back(byte);
                if (buffer_.size() - footerStartPos_ == footer.size())
                {
                    Frame frame{buffer_, currentType_};
                    reset();
                    return std::make_unique<Frame>(frame);
                }
            }
            else
            {
                buffer_.push_back(byte);
                state_ = ProcessorState::WaitingForFooterStart;
            }
            return nullptr;
        }
        return nullptr;
    }
    
    ProcessorState getState() const { return state_; }
    size_t getBufferSize() const { return buffer_.size(); }
    void reset()
    {
        state_ = ProcessorState::WaitingForHeaderStart;
        buffer_.clear();
        dataStartpos_ = 0;
        footerStartPos_ = 0;
        currentType_ = 0;
    }

private:
    ProcessorState state_ = ProcessorState::WaitingForHeaderStart;
    std::vector<uint8_t> buffer_;
    size_t footerStartPos_ = 0;
    size_t dataStartpos_ = 0;
    uint8_t currentType_ = 0;

    const std::unordered_map<uint8_t, HeaderDataFooter> &headerDataFooterMap_;
};
