/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "packageReceiver.hpp"
#include "millipede.hpp"

int main(int argc, char const *argv[])
{
    Config *cfg = new Config("data/tddconfig.json");
    Millipede *millipede_cli = new Millipede(cfg);
    millipede_cli->start();

    return 0;
}
