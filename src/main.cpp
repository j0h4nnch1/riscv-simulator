#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "common/option_parser.h"
#include "sim/sim.h"

static void help(int ext_code = 1){
    printf("help\n");
    exit(ext_code);
}
// static void update_img(const char* file){
//     printf("update_img:%s\n", file);
//     int ext_code = 1;
//     exit(ext_code);
// }
 
int main(int argc, char** argv){
    sim_t sim; 
    option_parser_t parser;
    parser.add_option('h', "help", [&](const char UNUSED *s){help(0);});
    parser.add_option('i', "img", [&](const char UNUSED *s){parser.update_img(sim, s);});
    printf("before run:%s\n",sim.file);
    sim.run();
    // int return_code = sim.run();
    parser.parse(argv);
    int return_code = 1;
    return return_code;
}