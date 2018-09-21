#include "Pins.h"

int sysconfgGPIOEdge(int pinBCM, std::string mode)
{
    cout << "\rConfiguring pin " << pinBCM << " in mode '" << mode << "'...";
    std::stringstream ss;
    ss << "gpio edge " << pinBCM << " " << mode;
    
    if (system(ss.str().c_str())!=0)
        return -1;

    cout << "\r...ok" << endl;
    return 0;
}
