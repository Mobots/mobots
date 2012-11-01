
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int gen_checksum(int argc, char **argv) {
    uint32_t buf[128];
    int res,rem = 0;
    uint32_t sum = 0;
    int count = 0;
    while ((res = fread(((char*)buf)+rem,1,512-rem,stdin)) != 0) {
	res += rem;
	rem = res & 3;
	res /= 4;
	int i;
	for(i = 0; i < res; i++) {
	    sum += buf[i];
	    count++;
	}
	if (rem > 0 && res != 0)
	    memmove(buf+0,buf+res,rem);
    }
    switch(rem) {
    case 0: break;
    case 1: sum += buf[0] & 0x000000ff; break;
    case 2: sum += buf[0] & 0x0000ffff; break;
    case 3: sum += buf[0] & 0x00ffffff; break;
    }
    printf("static __attribute__((section(\".checksum\"),used)) const int __checksum = 0x%08x;\n",-sum);
    if (ferror(stdin) || ferror(stdout))
	return 1;
    return 0;
}
