// This file is part of papo-model
//
// papo-model is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// papo-model is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Author: Julian Kunkel

#ifndef OPTION_PARSER_H
#define OPTION_PARSER_H

typedef enum{
  OPTION_FLAG,
  OPTION_OPTIONAL_ARGUMENT,
  OPTION_REQUIRED_ARGUMENT
} option_value_type;

typedef struct{
  char shortVar;
  char * longVar;
  char * help;

  option_value_type arg;
  char type;  // data type, H = hidden string
  void * variable;
} option_help;

#define LAST_OPTION {0, 0, 0, (option_value_type) 0, 0, NULL}

void option_print_help(option_help * args, const char * prefix);
void option_print_current_values(option_help * args);

//@return the number of parsed arguments
int option_parseOptions(int argc, char ** argv, option_help * args, int * print_help, int print_errors);

#endif
