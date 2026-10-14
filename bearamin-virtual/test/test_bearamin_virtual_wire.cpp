#include "../firmware/common/bearamin_virtual_wire.h"
#include <cassert>
#include <cstring>

int main() {
    int16_t mm[64]{};
    uint8_t status[64]{};
    for (int i=0;i<64;++i) { mm[i]=421; status[i]=5; }
    mm[0]=0; mm[1]=2001; status[2]=3; status[3]=9; mm[3]=2000;
    char line[1024]{};
    size_t n=bearamin_virtual::frameLine(line,sizeof(line),123,42,2800042,mm,status);
    assert(n>0 && n<1024 && line[n-1]=='\n');
    assert(std::strstr(line,"\"mm\":[null,null,null,2000,421") != nullptr);
    assert(std::strstr(line,"\"boot\":123,\"seq\":42,\"capture_us\":2800042") != nullptr);
    assert(bearamin_virtual::frameLine(line,40,123,42,1,mm,status)==0);
    n=bearamin_virtual::statusLine(line,sizeof(line),1,2,3,true);
    assert(n>0 && std::strstr(line,"\"state\":\"sensor_fault\"") != nullptr);
    assert(std::strstr(line,"\"mm\"")==nullptr);
}
