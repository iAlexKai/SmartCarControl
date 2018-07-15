#include <iostream>
#include <string>
#define READ_PIN 1
#include <unistd.h>
using namespace std;

int read_pin(string pin_BCM_number) {
    char buf[5];
    FILE* f = popen(("gpio -g read " + pin_BCM_number).c_str(), "r");
    if (f == NULL) {
        cout << "failed" << endl;
        exit(0);
    }

    fgets(buf, 5, f);
    pclose(f);
    return buf[0] - '0';
}

void write_pin(string pin_BCM_number, string val) {
    char buf[5];
    system(("gpio -g write " + pin_BCM_number+ " " + val).c_str());
}

void set_pin_mode(string pin_BCM_number, string mode) {
    char buf[5];
    system(("gpio -g mode " + pin_BCM_number + " " + mode).c_str());
}

int main() {
    system("gpio readall");
    //cout << "read 17 " << read_pin("17") << endl;
    //cout << "read 6 " << read_pin("6") << endl;

    set_pin_mode("4", "out");
    //system("gpio readall");
    for (int i=0; i < 100; i++) {
            write_pin("4", "1");
            sleep(1);
            cout << i << endl;
            //system("gpio readall");
            write_pin("4", "0");
            sleep(1);
    }
    set_pin_mode("4", "in");

    //cout << "read " << read_pin(int pin_BCM_number) << endl;
    return 0;
}

