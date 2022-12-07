//
// Created by Roger Massana on 11/5/22.
//
#include "winsuport2.h"
#include "memoria.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "semafor.h"
#include "missatge.h"
#include <pthread.h>

#define BLKCHAR 'B'
#define FRNTCHAR 'A'
#define MAXBALLS 9
#define BLKT 'T'
#define BLKHASH '#'
float pos_f[MAXBALLS], pos_c[MAXBALLS], vel_f[MAXBALLS], vel_c[MAXBALLS];
int f_pil[MAXBALLS], c_pil[MAXBALLS], *pilotes_en_joc, idpilotes, n_fil, n_col, m_pal, *c_pal, idpal, *nblocs, idblocs, retard, fi1 = 0,
    fi2 = 0, idIn, in, *pIn, *p_estat, npilotes;
int id_semafor, id_bustia;
float velNova = 0;

pid_t pid;
pthread_t tid;
char stringArg[20][100];

float valor_abs(float a){
    return a < 0 ? -a : a;
}

void* comprovar_bustia(void* a){
    int i = 0;
    float missatge = 0;
    while (i < npilotes){
        receiveM(id_bustia, &missatge);
        missatge = valor_abs(missatge);
        if (valor_abs(vel_f[in]) < missatge){
            missatge = missatge + missatge * 0.25;
            vel_f[in] = missatge;
        } else
            vel_f[in] = missatge;
        if (vel_f[in] > 1)
            vel_f[in] = 1;
        else if (vel_f[in] < -1)
            vel_f[in] = -1;
        i++;
    }
    return 0;
}

float control_impacte2(int col_pil, float velc0) {
    int distApal;
    float vel_col;
 
    distApal = col_pil - *c_pal;
    if (distApal >= 2 * m_pal / 3)    /* costat dreta */
        vel_col = 0.5;
    else if (distApal <= m_pal / 3)    /* costat esquerra */
        vel_col = -0.5;
    else if (distApal == m_pal / 2)    /* al centre */
        vel_col = 0.0;
    else /*: rebot normal */
        vel_col = velc0;
    return vel_col;
     
}

/* funcio per moure la pilota: retorna un 1 si la pilota surt per la porteria,*/
/* altrament retorna un 0 */
/* Si hi ha una col.lisió pilota-bloci esborra el bloc */
void comprovar_bloc(int f, int c) {
    int col;
    waitS(id_semafor); /* tanca semafor */
    char quin = win_quincar(f, c);
    signalS(id_semafor); /* obre semafor */
    if (quin == BLKCHAR || quin == FRNTCHAR || quin == BLKT) {
        col = c;
        waitS(id_semafor); /* tanca semafor */
        while (win_quincar(f, col) != ' ') {
            win_escricar(f, col, ' ', NO_INV);
            col++;
        }
        signalS(id_semafor); /* obre semafor */
        col = c - 1;
        waitS(id_semafor); /* tanca semafor */
        while (win_quincar(f, col) != ' ') {
            win_escricar(f, col, ' ', NO_INV);
            col--;
        }
        signalS(id_semafor); /* obre semafor */

        if (quin == BLKCHAR && in < npilotes - 1) {
            //fprintf(stderr,"numero pilotes joc: %d", *pilotes_en_joc);
            fprintf(stderr,"numero pilotes joc: %d\n", *pilotes_en_joc);
            fprintf(stderr,"index: %d\n", *pIn);
            waitS(id_semafor); /* tanca semafor */
            *pIn = *pIn + 1;
            (*pilotes_en_joc)++;
            /* generar la pilota */
            win_escricar(f_pil[in], c_pil[in], '1', INVERS);
            signalS(id_semafor); /* obre semafor */
            pid = fork();
            
        
            if (pid == (pid_t) 0) {
                execlp("./pilota4", stringArg[0], stringArg[1], stringArg[2], stringArg[3], stringArg[4],
                       stringArg[5], stringArg[6], stringArg[7], stringArg[8], stringArg[9], stringArg[10],
                       stringArg[11], stringArg[12], stringArg[13], stringArg[14], stringArg[15],
                       stringArg[16], stringArg[17], stringArg[18], stringArg[19],(char *) 0);
                exit(0);
            }
        }
        (*nblocs)--;
        //fprintf(stderr,"nblocs:%d \n", *nblocs);
        if (quin == BLKT){
            waitS(id_semafor); /* tanca semafor */
            (*p_estat)++;
            signalS(id_semafor); /* obre semafor */
        }
    } else if (quin == BLKHASH && *p_estat) {
        waitS(id_semafor); /* tanca semafor */
        win_escricar(f, c, ' ', NO_INV);
        signalS(id_semafor); /* obre semafor */
    }
}

void mou_pilota() {
    int f_h, c_h;
    char rh, rv, rd;
    pthread_create(&tid, NULL, comprovar_bustia, NULL);
    do {
        f_h = pos_f[in] + vel_f[in];    /* posicio hipotetica de la pilota (entera) */
        c_h = pos_c[in] + vel_c[in];
        rh = rv = rd = ' ';
        if ((f_h != f_pil[in]) || (c_h != c_pil[in])) {
            /* si posicio hipotetica no coincideix amb la posicio actual */
            if (f_h != f_pil[in]) {    /* provar rebot vertical */
                waitS(id_semafor); /* tanca semafor */
                rv = win_quincar(f_h, c_pil[in]);    /* veure si hi ha algun obstacle */
                signalS(id_semafor); /* obre semafor */
                if (rv != ' ') {    /* si hi ha alguna cosa */
                
                    comprovar_bloc(f_h, c_pil[in]);

                    if (rv == '0') {    /* col.lisió amb la paleta? */
                        //					control_impacte();
                        waitS(id_semafor);
                        vel_c[in] = control_impacte2(c_pil[in], vel_c[in]);
                        signalS(id_semafor);
                    }
                    vel_f[in] = -vel_f[in];    /* canvia sentit velocitat vertical */
                    f_h = pos_f[in] + vel_f[in];    /* actualitza posicio hipotetica */
                }
            }
            if (c_h != c_pil[in]) {    /* provar rebot horitzontal */
                waitS(id_semafor); /* tanca semafor */
                rh = win_quincar(f_pil[in], c_h);    /* veure si hi ha algun obstacle */
                signalS(id_semafor); /* obre semafor */

                if (rh != ' ') {    /* si hi ha algun obstacle */
                    comprovar_bloc(f_pil[in], c_h);

                    /* TODO?: tractar la col.lisio lateral amb la paleta */
                    vel_c[in] = -vel_c[in];    /* canvia sentit vel. horitzontal */
                    c_h = pos_c[in] + vel_c[in];    /* actualitza posicio hipotetica */
                }
            }
            if ((f_h != f_pil[in]) && (c_h != c_pil[in])) {    /* provar rebot diagonal */
                waitS(id_semafor); /* tanca semafor */
                rd = win_quincar(f_h, c_h);
                signalS(id_semafor); /* obre semafor */

                if (rd != ' ') {    /* si hi ha obstacle */
                    comprovar_bloc(f_h, c_h);

                    vel_f[in] = -vel_f[in];
                    vel_c[in] = -vel_c[in];    /* canvia sentit velocitats */
                    f_h = pos_f[in] + vel_f[in];
                    c_h = pos_c[in] + vel_c[in];    /* actualitza posicio entera */
                }
            }
            /* mostrar la pilota a la nova posició */
            waitS(id_semafor); /* tanca semafor */
            if (win_quincar(f_h, c_h) == ' ') {    /* verificar posicio definitiva *//* si no hi ha obstacle */
                win_escricar(f_pil[in], c_pil[in], ' ', NO_INV);    /* esborra pilota */
                signalS(id_semafor); /* obre semafor */

                pos_f[in] += vel_f[in];
                pos_c[in] += vel_c[in];
                f_pil[in] = f_h;
                c_pil[in] = c_h;    /* actualitza posicio actual */
                if (f_pil[in] != n_fil - 1) {/* si no surt del taulell, */
                    waitS(id_semafor); /* tanca semafor */
                    win_escricar(f_pil[in], c_pil[in], '1', *p_estat ? NO_INV : INVERS);    /* imprimeix pilota */
                    signalS(id_semafor); /* obre semafor */
                } else {
                    waitS(id_semafor); /* tanca semafor */
                    for (int i = 0; i < *pIn; i++) {
                        sendM(id_bustia,&(vel_f[in]), sizeof(int));
                    }
                    fprintf(stderr, "resta de pilotes en joc: %d\n",*pilotes_en_joc);
                    *pilotes_en_joc = *pilotes_en_joc - 1;
                    signalS(id_semafor); /* obre semafor */
                }
            } else {
                signalS(id_semafor); /* obre semafor */
            }
        } else {    /* posicio hipotetica = a la real: moure */
            pos_f[in] += vel_f[in];
            pos_c[in] += vel_c[in];
        }
        win_retard(retard);

    } while ( *nblocs > 0 && *pilotes_en_joc > 0 );
}

int main(int n_args, char *ll_args[]) {

    //fprintf(stderr, "nargs: %d", n_args);
    npilotes = atoi(ll_args[8]);
    n_fil = atoi(ll_args[2]);
    n_fil++;
    n_col = atoi(ll_args[3]);
    m_pal = atoi(ll_args[11]);
    idpal = atoi(ll_args[12]);
    idblocs = atoi(ll_args[13]);
    retard = atoi(ll_args[14]);
    idIn = atoi(ll_args[15]);
    idpilotes = atoi(ll_args[19]);
    //idpilotes = atoid(ll_args[18]);

    pIn = (int *)map_mem(idIn);
    in = *pIn;

    nblocs = (int*)map_mem(idblocs);

    pilotes_en_joc = (int*)map_mem(idpilotes);
    
    
    //fprintf(stderr," npilotes dins de pilota4: %d",npilotes);
    //fprintf(stderr," pilotes_en_joc dins de pilota4: %d", *pilotes_en_joc);

    c_pal = (int*)map_mem(idpal);
    //copiem la taula de string

    for (int i = 0; i < n_args; i++) {
        strcpy(stringArg[i], ll_args[i]);
        //fprintf(stderr, "StringArg: %s\n", stringArg[i]);
    }

    int mem = atoi(ll_args[1]);
    int *p_camp;
    p_camp = (int *) map_mem(mem);
    win_set((void *) (intptr_t) p_camp, n_fil, n_col);

    int id_estat = atoi(ll_args[18]);
    p_estat = (int *) map_mem(id_estat);

    id_semafor = atoi(ll_args[16]); /* obtenir identificador de semafor */
    id_bustia = atoi(ll_args[17]); /* obtenir identificador de bustia */

    if (p_camp == (int *) -1) {
        fprintf(stderr, "proces (%d): error en identificador de memoria\n", getpid());
        exit(0);
    }

    int i = 0;
    char *tok = strtok(ll_args[4], ", ");
    while (tok != NULL) {
        pos_f[i] = atof(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[5], ", ");

    i = 0;

    while (tok != NULL) {
        pos_c[i] = atof(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[6], ", ");
    i = 0;

    while (tok != NULL) {
        vel_f[i] = atof(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[7], ", ");
    i = 0;

    while (tok != NULL) {
        vel_c[i] = atof(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[9], ", ");
    i = 0;

    while (tok != NULL) {
        f_pil[i] = atoi(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    tok = strtok(ll_args[10], ", ");
    i = 0;

    while (tok != NULL) {
        c_pil[i] = atoi(tok);
        tok = strtok(NULL, ", ");
        i++;
    }

    //fprintf(stderr," npilotes dins de pilota4: %d",npilotes);
    //fprintf(stderr," pilotes_en_joc dins de pilota4: %d", *pilotes_en_joc);
    mou_pilota();

    return 1;
}

