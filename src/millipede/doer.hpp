#ifndef DOER
#define DOER

class Config;
class Consumer;

class Doer {
public:
    Doer(Config* in_config, int in_tid, Consumer& in_consumer)
        : config_(in_config)
        , tid(in_tid)
        , consumer_(in_consumer)
    {
    }

protected:
    Config* config_;
    int tid;
    Consumer& consumer_;
};
#endif /* DOER */
