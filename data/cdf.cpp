#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <algorithm>

using namespace std;

size_t n_data = 0;
bool setminv = false, setmaxv = false;
double minv, maxv;
size_t n_bins = 101;

void init(int argc, char **argv) {
    int opterr = 0;
    char c;

    while ((c = getopt (argc, argv, "b:m:x:")) != -1) {
        switch (c) {
            case 'b':
                n_bins = atoi(optarg);
                break;
            case 'm':
                setminv = true;
                minv = atof(optarg);
                break;
            case 'x':
                setmaxv = true;
                maxv = atof(optarg);
                break;
            case '?':
                if (optopt == 'b' || optopt == 'm' || optopt == 'x')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr,
                            "Unknown option character `\\x%x'.\n",
                            optopt);
                return;
            default:
                abort ();
        }
    }
}

int main(int argc, char **argv)
{
    init(argc, argv);

    vector<double> data;
    double tmp;
    while (cin >> tmp) {
        data.push_back(tmp);
        n_data ++;
    }

    sort(data.begin(), data.end());

    if (!setminv) {
        minv = data[0];
    }
    if (!setmaxv) {
        maxv = data[data.size()-1];
    }

    vector<double> cdf;
    cdf.resize(n_bins);
    double size_bin = (maxv - minv) / (n_bins - 1);

    for (double value : data) {
        int idx = (value - minv) / size_bin;
        if (idx > 0 && idx < n_bins) {
            cdf[idx] ++;
        }
    }

    for (size_t i = 1; i < cdf.size(); i ++) {
        cdf[i] += cdf[i-1];
    }

    for (size_t i = 0; i < cdf.size(); i ++) {
        cdf[i] /= n_data;
        printf("%lf %lf\n", minv + i * size_bin, 1.0 - cdf[i]);
    }

    return 0;
}