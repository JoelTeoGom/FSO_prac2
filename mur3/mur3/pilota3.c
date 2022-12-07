//
// Created by Roger Massana on 11/5/22.
//
#include "winsuport2.h"
#include "memoria.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define BLKCHAR 'B'
#define FRNTCHAR 'A'
#define MAXBALLS 9
float pos_f[MAXBALLS], pos_c[MAXBALLS], vel_f[MAXBALLS], vel_c[MAXBALLS];
int f_pil[MAXBALLS], c_pil[MAXBALLS], npilotes, n_fil, n_col, m_pal, *c_pal, idpal, *nblocs, idblocs,  retard,  idIn, in, *pIn, idFi, *pFi;
pid_t pid;
char stringArg[17][100];

float control_impacte2(int col_pil, float velc0) {
    int distApal;
    float vel_col;

    distApal = col_pil - *c_pal;
    fprintf(stderr, "valor inicial de paleta: %d en el process [%d] \n", *c_pal, in);

    
    /*
    fprintf(stderr, "===================================================================================================================\n");
    fprintf(stderr, "valor inicial de paleta: %d\n", *c_pal);
    fprintf(stderr, "===================================================================================================================\n");
    fprintf(stderr, "valor c de pilota: %d\n", col_pil);*/
    if (distApal >= 2*m_pal/3)	/* costat dreta */
        vel_col = 0.5;
    else if (distApal <= m_pal/3)	/* costat esquerra */
        vel_col = -0.5;
    else if (distApal == m_pal/2)	/* al centre */
        vel_col = 0.0;
    else /*: rebot normal */
        vel_col = velc0;
    return vel_col;
}

/* funcio per moure la pilota: retorna un 1 si la pilota surt per la porteria,*/
/* altrament retorna un 0 */
/* Si hi ha una col.lisió pilota-bloci esborra el bloc */
void comprovar_bloc(int f, int c)
{
    int col;
    char quin = win_quincar(f, c);

    if (quin == BLKCHAR || quin == FRNTCHAR) {
        col = c;
        while (win_quincar(f, col) != ' ') {
            win_escricar(f, col, ' ', NO_INV);
            col++;
        }
        col = c - 1;
        while (win_quincar(f, col) != ' ') {
            win_escricar(f, col, ' ', NO_INV);
            col--;
        }

        if (quin == BLKCHAR && in < npilotes){
            *pIn = *pIn + 1;
            /* generar la pilota */
            win_escricar(f_pil[in], c_pil[in], '1', INVERS);
            pid = fork();
            if (pid == (pid_t) 0){
                //fprintf(stderr, "oejwaopefjawopeijfa");
                execlp("./pilota3", stringArg[0], stringArg[1], stringArg[2], stringArg[3], stringArg[4],
                       stringArg[5], stringArg[6], stringArg[7], stringArg[8], stringArg[9], stringArg[10], stringArg[11],
                       stringArg[12], stringArg[13], stringArg[14], stringArg[15], stringArg[16],(char *) 0);
                exit(0);
            }

        }
        (*nblocs)--;
    }
}

void mou_pilota()
{
    int f_h, c_h;
    char rh, rv, rd;

    do{
        f_h = pos_f[in] + vel_f[in];	/* posicio hipotetica de la pilota (entera) */
        c_h = pos_c[in] + vel_c[in];
        rh = rv = rd = ' ';
        if ((f_h != f_pil[in]) || (c_h != c_pil[in])) {
            /* si posicio hipotetica no coincideix amb la posicio actual */
            if (f_h != f_pil[in]) {	/* provar rebot vertical */
                rv = win_quincar(f_h, c_pil[in]);	/* veure si hi ha algun obstacle */
                if (rv != ' ') {	/* si hi ha alguna cosa */
                    comprovar_bloc(f_h, c_pil[in]);

                    if (rv == '0') {    /* col.lisió amb la paleta? */
                        //					control_impacte();
                        vel_c[in] = control_impacte2(c_pil[in], vel_c[in]);
                    }
                    vel_f[in] = -vel_f[in];	/* canvia sentit velocitat vertical */
                    f_h = pos_f[in] + vel_f[in];	/* actualitza posicio hipotetica */
                }
            }
            if (c_h != c_pil[in]) {	/* provar rebot horitzontal */
                rh = win_quincar(f_pil[in], c_h);	/* veure si hi ha algun obstacle */

                if (rh != ' ') {	/* si hi ha algun obstacle */
                    comprovar_bloc(f_pil[in], c_h);
                    /* TODO?: tractar la col.lisio lateral amb la paleta */
                    vel_c[in] = -vel_c[in];	/* canvia sentit vel. horitzontal */
                    c_h = pos_c[in] + vel_c[in];	/* actualitza posicio hipotetica */
                }
            }
            if ((f_h != f_pil[in]) && (c_h != c_pil[in])) {	/* provar rebot diagonal */
                rd = win_quincar(f_h, c_h);

                if (rd != ' ') {	/* si hi ha obstacle */
                    comprovar_bloc(f_h, c_h);
                    vel_f[in] = -vel_f[in];
                    vel_c[in] = -vel_c[in];	/* canvia sentit velocitats */
                    f_h = pos_f[in] + vel_f[in];
                    c_h = pos_c[in] + vel_c[in];	/* actualitza posicio entera */
                }
            }
            /* mostrar la pilota a la nova posició */
            if (win_quincar(f_h, c_h) == ' ') {	/* verificar posicio definitiva *//* si no hi ha obstacle */
                win_escricar(f_pil[in], c_pil[in], ' ', NO_INV);	/* esborra pilota */
                pos_f[in] += vel_f[in];
                pos_c[in] += vel_c[in];
                f_pil[in] = f_h;
                c_pil[in] = c_h;	/* actualitza posicio actual */
                if (f_pil[in] != n_fil - 1) {/* si no surt del taulell, */
                    win_escricar(f_pil[in], c_pil[in], '1', INVERS);    /* imprimeix pilota */
                } else
                    *(pFi) = 1;

            }
        } else {	/* posicio hipotetica = a la real: moure */
            pos_f[in] += vel_f[in];
            pos_c[in] += vel_c[in];
        }
        
        if((*nblocs) == 0){
            *(pFi) = 1;
        }

        win_retard(retard);

    } while (*(pFi) == 0);
}

int main(int n_args, char *ll_args[]){

    npilotes = atoi(ll_args[8]);
    n_fil = atoi(ll_args[2]);
    n_fil++;
    n_col = atoi(ll_args[3]);
    m_pal = atoi(ll_args[11]);
    idpal = atoi(ll_args[12]);
    idblocs = atoi(ll_args[13]);
    retard = atoi(ll_args[14]);
    idIn = atoi(ll_args[15]);
    idFi = atoi(ll_args[16]);
    

    pIn = (int*) map_mem(idIn);
    in = *pIn;

    pFi = (int*) map_mem(idFi);
 

    nblocs = (int*)map_mem(idblocs);

    c_pal = (int*)map_mem(idpal);


    //copiem la taula de string

    for (int i = 0; i < n_args; i++){
        strcpy(stringArg[i], ll_args[i]);
       // fprintf(stderr, "StringArg: %s\n", stringArg[i]);
    }


    int mem = atoi(ll_args[1]);
    int *p_camp;
    p_camp = (int*) map_mem(mem);
    win_set((void*) (intptr_t)p_camp, n_fil, n_col);

    if (p_camp == (int*) -1)
    {
        //fprintf(stderr,"proces (%d): error en identificador de memoria\n", getpid());
        exit(0);
    }

    int i = 0;
    char *tok = strtok(ll_args[4], ", ");
    while(tok != NULL)
    {
        pos_f[i] = atof(tok);
        //fprintf(stderr, "Pos_f: %s\n", tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[5], ", ");

    i = 0;

    while(tok != NULL)
    {
        pos_c[i] = atof(tok);
        //fprintf(stderr, "Pos_c: %s\n", tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[6], ", ");
    i = 0;

    while(tok != NULL)
    {
        vel_f[i] = atof(tok);
        //fprintf(stderr, "Vel_f: %s", tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[7], ", ");
    i = 0;

    while(tok != NULL)
    {
        vel_c[i] = atof(tok);
        //fprintf(stderr, "Vel_c: %s", tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[9], ", ");
    i = 0;

    while(tok != NULL)
    {
        f_pil[i] = atoi(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[10], ", ");
    i = 0;

    while(tok != NULL)
    {
        c_pil[i] = atoi(tok);
        tok = strtok(NULL, ", ");
        i++;
    }
    mou_pilota();

    return 0;
}

