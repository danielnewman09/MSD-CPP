
class BaseTransferObject
{
public:
    explicit BaseTransferObject(auto integral id)
        : id_(id)
    {
    }

private:
    uint32_t id_;

};