#include <stdio.h>
#include <stdlib.h>

#ifndef SIZE
#define SIZE 10
#endif
#define ERROR_NAME 101
#define ERROR_OPEN 102
#define ERROR_MEMORY 103
#define ERROR_CLOSE 104

//counting length of string
unsigned long myStrLen(const char *str){
    unsigned long length = 0;
    while(str && str[length] != '\0'){
        length += 1;
    }
    return length;
}

// reading chars til end of line => '\n'
char *readLine(FILE *fid){
    int capacity = SIZE;
    char *line = malloc(capacity + 1);
    int length = 0;
    int r;
    if (!line){
        fprintf(stderr,"Error: Pamet sa neda alokovat\n");
        exit(ERROR_MEMORY);
    }    
    while ((r = getc(fid)) && r!= EOF && r != '\n'){
        //enlarge memory for line
        if(length== capacity){     
            char *t = realloc(line, capacity + SIZE + 1);
            if (t == NULL){
                free(line);
                fprintf(stderr,"Error: Nepodarilo sa realokovat pamet\n");
                exit(ERROR_MEMORY);
            }
            line = t;
            capacity += SIZE; 
        } // end of enlarging capacity
        line[length++] = r;
    } // end of reading characters
    
    if (r == EOF && length == 0){
        free(line);
        line = NULL;
    }
    else{
        line[length] = '\0';
    }
    return line;
}



//comparing chars if there is match then match_index => index in line where pattern starts
int strMatch(const char *pattern, const char *line){
    int match_index = -1; 
    if (pattern && line){
        int index1 = 0; 
        int index2 = 0;
        int start = 0;
        while(pattern[index1] != '\0' && line[index2] != '\0'){
            if (pattern[index1] != line[index2])
                index1 = 0;
            if(pattern[index1] == line[index2]){
                if (index1 == 0){
                    start = index2;
                }
                index1 +=1;
            }    
            index2 += 1;
        }
        if (pattern[index1] == '\0'){
            match_index = start;
        }
    }
    return match_index;
}

// for option --color=always printing line with red color for pattern occurrence
void printline(const char *pattern,const char *line){
  int len1 = myStrLen(pattern);
  int index1 = 0;
  int index2 = 0;
  int match = 0;
  while(line[index2] != '\0'){ //until end of line
    if (pattern[index1] == line[index2]) { 
      for(int j = 0; j < len1; ++j){       //checking if the match is for whole pattern
        if (pattern[j] != line[index2+j]) {
          match = -1;
          break;
        }
      }
      if (match == 0){
        printf("\033[01;31m\033[K");    //red color before text
        printf("%s",pattern);
        printf("\033[m\033[K");         // red color after text
        index2 += len1;
        match = 0;
      }
    }
    else{
      printf("%c",line[index2]);
      index2 += 1;
    }
  }
}


int main(int argc,char *argv[]){
    int ret = 1;
    int arg = 0; 
    int lenarg = 0;
    // for option C
    if (argc > 3){
      const char *arg1 = argc > 1 ? argv[1] : NULL;
      lenarg = myStrLen(arg1);
      arg += 1;
    }

    const char *pattern = argc > (1+arg) ? argv[(1+arg)] : NULL;
    const char *fname = argc > (2+arg) ? argv[(2+arg)] : NULL; 

    if (!pattern || !fname){
        fprintf(stderr, "Error: Chybi meno souboru jako argument\n");
        return ERROR_NAME;
    }
    FILE *fid = fopen(fname, "r");
    if (fid == NULL){
        fprintf(stderr, "Error: Soubor se nepodarilo otevrit.\n");
        return ERROR_OPEN;
    }

    char *line = readLine(fid);
    while (line != NULL){
        int match = strMatch(pattern, line);
        if (match >= 0){
            if (arg == 0){
              printf("%s\n", line);
              ret = 0;
            } 
            else if (lenarg >0){
                printline(pattern,line);
                putchar('\n');
                ret = 0;
            }
        }
        free(line);
        line = NULL;
        line = readLine(fid);
    }
    if (fclose(fid) == EOF) {
        fprintf(stderr, "Soubor se nepodarilo uzavrit.\n");
        exit(ERROR_CLOSE);
    }  

    return ret;
}
