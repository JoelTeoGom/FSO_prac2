/*****************************************************************************/
/*                                                                           */
/*                           mur0.c                                          */
/*                                                                           */
/*  Programa inicial d'exemple per a les practiques 2 de FSO.                */
/*                                                                           */
/*  Compilar i executar:                                                     */
/*     El programa invoca les funcions definides a "winsuport.c", les        */
/*     quals proporcionen una interficie senzilla per crear una finestra     */
/*     de text on es poden escriure caracters en posicions especifiques de   */
/*     la pantalla (basada en CURSES); per tant, el programa necessita ser   */
/*     compilat amb la llibreria 'curses':                                   */
/*                                                                           */
/*       $ gcc -Wall -c winsuport.c -o winsuport.o                           */
/*       $ gcc -Wall mur0.c winsuport.o -o mur0 -lcurses                     */
/*                                                                           */
/*  Al tenir una orientació vertical cal tenir un terminal amb prou files.   */
/*  Per exemple:                                                             */
/*               xterm -geometry 80x50                                       */
/*               gnome-terminal --geometry 80x50                             */
/*                                                                           */
/*****************************************************************************/

//#include <stdint.h>		/* intptr_t for 64bits machines */

#include <stdio.h>		/* incloure definicions de funcions estandard */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "winsuport2.h"		/* incloure definicions de funcions propies */
#include <pthread.h>
#include <stdint.h>
#include <time.h>
#include "memoria.h"

/* definicio de constants */
#define MAX_THREADS	10
#define MAXBALLS	(MAX_THREADS-1)
#define MIN_FIL	10		/* dimensions del camp. Depenen del terminal ! */
#define MAX_FIL	50
#define MIN_COL	10
#define MAX_COL	80
#define BLKSIZE	3
#define BLKGAP	2
#define BLKCHAR 'B'
#define WLLCHAR '#'
#define FRNTCHAR 'A'
#define LONGMISS	65
#define MAX_PROCS 10
/* variables globals */
char *descripcio[] = {
        "\n",
        "Aquest programa implementa una versio basica del joc Arkanoid o Breakout:\n",
        "generar un camp de joc rectangular amb una porteria, una paleta que s\'ha\n",
        "de moure amb el teclat per a cobrir la porteria, i una pilota que rebota\n",
        "contra les parets del camp, a la paleta i els blocs. El programa acaba si\n",
        "la pilota surt per la porteria o no queden mes blocs. Tambe es pot acabar\n",
        "amb la tecla RETURN.\n",
        "\n",
        "  Arguments del programa:\n",
        "\n",
        "       $ ./mur0 fitxer_config [retard]\n",
        "\n",
        "     El primer argument ha de ser el nom d\'un fitxer de text amb la\n",
        "     configuracio de la partida, on la primera fila inclou informacio\n",
        "     del camp de joc (valors enters), i la segona fila indica posicio\n",
        "     i velocitat de la pilota (valors reals):\n",
        "          num_files  num_columnes  mida_porteria\n",
        "          pos_fil_pal pos_col_pal mida_pal\n"
        "          pos_fila   pos_columna   vel_fila  vel_columna\n",
        "\n",
        "     on els valors minims i maxims admesos son els seguents:\n",
        "          MIN_FIL <= num_files     <= MAX_FIL\n",
        "          MIN_COL <= num_columnes  <= MAX_COL\n",
        "          0 < mida_porteria < num_files-2\n",
        "        1.0 <= pos_fila     <= num_files-3\n",
        "        1.0 <= pos_columna  <= num_columnes\n",
        "       -1.0 <= vel_fila     <= 1.0\n",
        "       -1.0 <= vel_columna  <= 1.0\n",
        "\n",
        "     Alternativament, es pot donar el valor 0 a num_files i num_columnes\n",
        "     per especificar que es vol que el tauler ocupi tota la pantalla. Si\n",
        "     tambe fixem mida_porteria a 0, el programa ajustara la mida d\'aquesta\n",
        "     a 3/4 de l\'altura del camp de joc.\n",
        "\n",
        "     A mes, es pot afegir un segon argument opcional per indicar el\n",
        "     retard de moviment del joc en mil.lisegons; el valor minim es 10,\n",
        "     el valor maxim es 1000, i el valor per defecte d'aquest parametre\n",
        "     es 100 (1 decima de segon).\n",
        "\n",
        "  Codis de retorn:\n",
        "     El programa retorna algun dels seguents codis:\n",
        "	0  ==>  funcionament normal\n",
        "	1  ==>  numero d'arguments incorrecte\n",
        "	2  ==>  no s\'ha pogut obrir el fitxer de configuracio\n",
        "	3  ==>  algun parametre del fitxer de configuracio es erroni\n",
        "	4  ==>  no s\'ha pogut crear el camp de joc (no pot iniciar CURSES)\n",
        "\n",
        "   Per a que pugui funcionar aquest programa cal tenir instal.lada la\n",
        "   llibreria de CURSES (qualsevol versio).\n",
        "\n",
        "*"
};				/* final de la descripcio */

int n_fil, n_col;		/* numero de files i columnes del taulell */
int m_por;			/* mida de la porteria (en caracters) */
int f_pal, *c_pal;		/* posicio del primer caracter de la paleta */
int m_pal;				/* mida de la paleta */
int f_pil[MAXBALLS], c_pil[MAXBALLS];		/* posicio de la pilota, en valor enter */
float pos_f[MAXBALLS], pos_c[MAXBALLS];		/* posicio de la pilota, en valor real */
float vel_f[MAXBALLS], vel_c[MAXBALLS];		/* velocitat de la pilota, en valor real */
int *nblocs, id_blocs;
int dirPaleta = 0;
int retard;			/* valor del retard de moviment, en mil.lisegons */
int fi2 = 0;
pthread_t tid[MAX_THREADS];
int n = 0;
int npilotes;
time_t temps_inicial, temps_actual;
char strin[LONGMISS];			/* variable per a generar missatges de text */
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; /* crea un sem. Global*/
pid_t tpid[MAX_PROCS]; /* taula d'identificadors dels processos fill */
int* p_camp, id_camp;   //Variables compartides
int mida_camp;
int *p_fi;

/* funcio per carregar i interpretar el fitxer de configuracio de la partida */
/* el parametre ha de ser un punter a fitxer de text, posicionat al principi */
/* la funcio tanca el fitxer, i retorna diferent de zero si hi ha problemes  */
int carrega_configuracio(FILE * fit)
{
    int ret = 0;

    fscanf(fit, "%d %d %d\n", &n_fil, &n_col, &m_por);	/* camp de joc */
    fscanf(fit, "%d %d %d\n", &f_pal, c_pal, &m_pal);	/* paleta */
    int i = 0;
    while (!feof(fit) && i < MAXBALLS){
        fscanf(fit, "%f %f %f %f\n", &pos_f[i], &pos_c[i], &vel_f[i], &vel_c[i]);	/* pilota */
        if ((n_fil != 0) || (n_col != 0)) {	/* si no dimensions maximes */
            if ((n_fil < MIN_FIL) || (n_fil > MAX_FIL) || (n_col < MIN_COL) || (n_col > MAX_COL))
                ret = 1;
            else if (m_por > n_col - 3)
                ret = 2;
            else if ((m_pal > n_col - 3) || (m_pal < 1) || (f_pal > n_fil-1) || (f_pal < 1) || (*c_pal + m_pal > n_col -1 || *c_pal < 1 ))
                ret = 3;
            else if ((pos_f[i] < 1) || (pos_f[i] >= n_fil - 3) || (pos_c[i] < 1)
                     || (pos_c[i] > n_col - 1))	/* tres files especials: línia d'estat, porteria i paleta */
                ret = 4;
        }
        if ((vel_f[i] < -1.0) || (vel_f[i] > 1.0) || (vel_c[i] < -1.0) || (vel_c[i] > 1.0))
            ret = 5;
        i++;
    }

    npilotes = i + 1;

    if (ret != 0) {		/* si ha detectat algun error */
        fprintf(stderr, "Error en fitxer de configuracio:\n");
        switch (ret) {
            case 1:
                fprintf(stderr,
                        "\tdimensions del camp de joc incorrectes:\n");
                fprintf(stderr, "\tn_fil= %d \tn_col= %d\n", n_fil,
                        n_col);
                break;
            case 2:
                fprintf(stderr, "\tmida de la porteria incorrecta:\n");
                fprintf(stderr, "\tm_por= %d\n", m_por);
                break;
            case 3:
                fprintf(stderr,"\tmida de la paleta incorrecta:\n");
                fprintf(stderr, "\tf_pal= %d \tc_pal= %d \t m_pal=%d\n", f_pal,*c_pal,m_pal);
                break;
            case 4:
                fprintf(stderr, "\tposicio de la pilota incorrecta:\n");
                fprintf(stderr, "\tpos_f= %.2f \tpos_c= %.2f\n", pos_f[0],
                        pos_c[0]);
                break;
            case 5:
                fprintf(stderr,
                        "\tvelocitat de la pilota incorrecta:\n");
                fprintf(stderr, "\tvel_f= %.2f \tvel_c= %.2f\n", vel_f[0],
                        vel_c[0]);
                break;
        }
    }
    fclose(fit);

    return (ret);
}

/* funcio per inicialitar les variables i visualitzar l'estat inicial del joc */
/* retorna diferent de zero si hi ha algun problema */
void inicialitza_joc(void)
{
    int i;
    int i_port, f_port;	/* inici i final de porteria */
    int c, nb, offset;

    mida_camp = win_ini(&n_fil, &n_col, '+', INVERS);	/* intenta crear taulell */

    if (mida_camp < 0) {	/* si no pot crear l'entorn de joc amb les curses */
        fprintf(stderr, "Error en la creacio del taulell de joc:\t");
        switch (mida_camp) {
            case -1:
                fprintf(stderr, "camp de joc ja creat!\n");
                break;
            case -2:
                fprintf(stderr,"no s'ha pogut inicialitzar l'entorn de curses!\n");
                break;
            case -3:
                fprintf(stderr,
                        "les mides del camp demanades son massa grans!\n");
                break;
            case -4:
                fprintf(stderr, "no s'ha pogut crear la finestra!\n");
                break;
        }
        return;
    }

    id_camp = ini_mem(mida_camp);
    p_camp = (int*) map_mem(id_camp);
    win_set((void*) (intptr_t) p_camp, n_fil, n_col);


    id_blocs = ini_mem(sizeof(int));
    nblocs = map_mem(id_blocs);

    if (m_por > n_col - 2)
        m_por = n_col - 2;	/* limita valor de la porteria */
    if (m_por == 0)
        m_por = 3 * (n_col - 2) / 4;	/* valor porteria per defecte */

    i_port = n_col / 2 - m_por / 2 - 1;	/* crea el forat de la porteria */
    f_port = i_port + m_por - 1;
    for (i = i_port; i <= f_port; i++)
        win_escricar(n_fil - 2, i, ' ', NO_INV);

    n_fil = n_fil - 1;	/* descompta la fila de missatges */

    for (i = 0; i < m_pal; i++) {    /* dibuixar paleta inicial */
        win_escricar(f_pal, *c_pal + i, '0', INVERS);
    }

    for (i = 0; i < npilotes; i++){
        /* generar la pilota */
        if (pos_f[i] > n_fil - 1)
            pos_f[i] = n_fil - 1;	/* limita posicio inicial de la pilota */
        if (pos_c[i] > n_col - 1)
            pos_c[i] = n_col - 1;
        f_pil[i] = pos_f[i];
        c_pil[i] = pos_c[i];
    }

    /* dibuixar la pilota inicialment */
    win_escricar(f_pil[0], c_pil[0], '1', INVERS);


    // generar els blocs
    nb = 0;
    *nblocs = n_col / (BLKSIZE + BLKGAP) - 1;
    offset = (n_col - *nblocs * (BLKSIZE + BLKGAP) + BLKGAP) / 2;
    for (i = 0; i < *nblocs; i++) {
        for (c = 0; c < BLKSIZE; c++) {
            win_escricar(3, offset + c, FRNTCHAR, INVERS);
            nb++;
            win_escricar(4, offset + c, BLKCHAR, NO_INV);
            nb++;
            win_escricar(5, offset + c, FRNTCHAR, INVERS);
            nb++;
        }
        offset += BLKSIZE + BLKGAP;
    }
    *nblocs = nb / BLKSIZE;
    // generar les defenses
    nb = n_col / (BLKSIZE + 2 * BLKGAP) - 2;
    offset = (n_col - nb * (BLKSIZE + 2 * BLKGAP) + BLKGAP) / 2;
    for (i = 0; i < nb; i++) {
        for (c = 0; c < BLKSIZE + BLKGAP; c++) {
            win_escricar(6, offset + c, WLLCHAR, NO_INV);
        }
        offset += BLKSIZE + 2 * BLKGAP;
    }
    sprintf(strin,
            "Tecles: \'%c\'-> Esquerra, \'%c\'-> Dreta, RETURN-> sortir\n",
            TEC_ESQUER, TEC_DRETA);
    win_escristr(strin);
}

/* funcio que escriu un missatge a la línia d'estat i tanca les curses */
void mostra_final(char *miss)
{	int lmarge;
    char marge[LONGMISS];

    /* centrar el misssatge */
    lmarge=(n_col+strlen(miss))/2;
    sprintf(marge,"%%%ds",lmarge);

    sprintf(strin, marge, miss);
    win_escristr(strin);
    win_update();
    /* espera tecla per a que es pugui veure el missatge */
    getchar();
}

/* funcio per a calcular rudimentariament els efectes amb la pala */
/* no te en compta si el moviment de la paleta no és recent */
/* cal tenir en compta que després es calcula el rebot */
void control_impacte(int index) {
    if (dirPaleta == TEC_DRETA) {
        if (vel_c[index] <= 0.0)	/* pilota cap a l'esquerra */
            vel_c[index] = -vel_c[index] - 0.2;	/* xoc: canvi de sentit i reduir velocitat */
        else {	/* a favor: incrementar velocitat */
            if (vel_c[index] <= 0.8)
                vel_c[index] += 0.2;
        }
    } else {
        if (dirPaleta == TEC_ESQUER) {
            if (vel_c[index] >= 0.0)	/* pilota cap a la dreta */
                vel_c[index] = -vel_c[index] + 0.2;	/* xoc: canvi de sentit i reduir la velocitat */
            else {	/* a favor: incrementar velocitat */
                if (vel_c[index] >= -0.8)
                    vel_c[index] -= 0.2;
            }
        } else {	/* XXX trucs no documentats */
            if (dirPaleta == TEC_AMUNT)
                vel_c[index] = 0.0;	/* vertical */
            else {
                if (dirPaleta == TEC_AVALL)
                    if (vel_f[index] <= 1.0)
                        vel_f[index] -= 0.2;	/* frenar */
            }
        }
    }
    dirPaleta=0;	/* reset perque ja hem aplicat l'efecte */
}

/* funcio per moure la paleta segons la tecla premuda */
/* retorna un boolea indicant si l'usuari vol acabar */
void* mou_paleta(void* nul)
{
    int tecla;
    do{
        tecla = win_gettec();
        if (tecla != 0) {
            if ((tecla == TEC_DRETA)
                && ((*c_pal + m_pal) < n_col - 1)) {
                win_escricar(f_pal, *c_pal, ' ', NO_INV);	/* esborra primer bloc */
                (*c_pal)++;	/* actualitza posicio */
                win_escricar(f_pal, *c_pal + m_pal - 1, '0', INVERS);	/*esc. ultim bloc */
            }
            if ((tecla == TEC_ESQUER) && (*c_pal > 1)) {
                win_escricar(f_pal, *c_pal + m_pal - 1, ' ', NO_INV);	/*esborra ultim bloc */
                (*c_pal)--;	/* actualitza posicio */
                win_escricar(f_pal, *c_pal, '0', INVERS);	/* escriure primer bloc */
            }
            if (tecla == TEC_RETURN)
                fi2 = 1;	/* final per pulsacio RETURN */
            dirPaleta = tecla;	/* per a afectar al moviment de les pilotes */
        }
        time(&temps_actual);
        double tempsResta = difftime(temps_actual, temps_inicial);
        int minuts = (int)tempsResta / 60;
        int segons = (int)tempsResta - minuts * 60;
        char timer[20];
        sprintf(timer, "[ %d : %d ]", minuts, segons);
        win_escristr(timer);
        win_retard(retard);
        win_update();
    } while(!(*p_fi) && !fi2);

    return (0);
}

/* programa principal */
int main(int n_args, char *ll_args[])
{
    time(&temps_inicial);
    int i;

    if (n_args == 3) {	/* si s'ha especificat parametre de retard */
        retard = atoi(ll_args[2]);	/* convertir-lo a enter */
        if (retard < 10)
            retard = 10;	/* verificar limits */
        if (retard > 1000)
            retard = 1000;
    } else
        retard = 100;	/* altrament, fixar retard per defecte */

    char a1[20], a2[5], a3[5], a4[100], a5[100], a6[100], a7[100], a8[10], a9[100], a10[100], a11[10], a12[10], a13[10], a14[10], a15[10],a16[10];
    char *aux4 = a4, *aux5 = a5, *aux6 = a6, *aux7 = a7, *aux9 = a9, *aux10 = a10;
    FILE *fit_conf;
    int idIn, *pIn, id_fi,id_pal;
    
    
    if ((n_args != 2) && (n_args != 3)) {	/* si numero d'arguments incorrecte */
        i = 0;
        do
            fprintf(stderr, "%s", descripcio[i++]);	/* imprimeix descripcio */
        while (descripcio[i][0] != '*');	/* mentre no arribi al final */
        exit(1);
    }

    idIn = ini_mem(sizeof (int));
    pIn = map_mem(idIn);
    *pIn = 0;

    id_fi = ini_mem(sizeof(int));
    p_fi = map_mem(id_fi);
    *p_fi = 0;

    id_pal = ini_mem(sizeof(int));
    c_pal = map_mem(id_pal);
    
    

    /*
    (*p_fi)[0]= 0;
    (*p_fi)[1]= 0;
    fprintf(stderr,"valor 0: %d, valor 1: %d",(*p_fi)[0],(*p_fi)[1]);
    */


    fit_conf = fopen(ll_args[1], "rt");	/* intenta obrir el fitxer */
    if (!fit_conf) {
        fprintf(stderr, "Error: no s'ha pogut obrir el fitxer \'%s\'\n",
                ll_args[1]);
        exit(2);
    }

    if (carrega_configuracio(fit_conf) != 0)	/* llegir dades del fitxer  */
        exit(3);/* aborta si hi ha algun problema en el fitxer */

    inicialitza_joc();


    sprintf(a1, "%d", id_camp); /* convertir id. memoria en string */
    sprintf(a2, "%d", n_fil);
    sprintf(a3, "%d", n_col);
    sprintf(a8, "%d", npilotes);
    sprintf(a11, "%d", m_pal);
    sprintf(a12, "%d", id_pal);
    sprintf(a13, "%d", id_blocs);
    sprintf(a14, "%d", retard);
    sprintf(a15, "%d", idIn);
    sprintf(a16, "%d", id_fi);



    for (i = 0 ; i < npilotes-1; i++) {
        if (i) {
            aux4 += sprintf(aux4, ", ");
            aux5 += sprintf(aux5, ", ");
            aux6 += sprintf(aux6, ", ");
            aux7 += sprintf(aux7, ", ");
            aux9 += sprintf(aux9, ", ");
            aux10 += sprintf(aux10, ", ");
        }
        aux4 += sprintf(aux4, "%f", pos_f[i]);
        aux5 += sprintf(aux5, "%f", pos_c[i]);
        aux6 += sprintf(aux6, "%f", vel_f[i]);
        aux7 += sprintf(aux7, "%f", vel_c[i]);
        aux9 += sprintf(aux9, "%d", f_pil[i]);
        aux10 += sprintf(aux10, "%d", c_pil[i]);
    }

    printf("Joc del Mur: prem RETURN per continuar:\n");
    getchar();
    pthread_create(&tid[0], NULL, mou_paleta, NULL);
    tpid[0] = fork();
    if (tpid[n] == (pid_t) 0){
        execlp("./pilota3", "pilota3", a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, (char *) 0);
        exit(0);
    }

    pthread_join(tid[0], NULL);

    while (!(*p_fi) && !fi2){
        win_retard(100);
        win_update();
    }

    

    if (*nblocs == 0)
        mostra_final("YOU WIN !");
    else
        mostra_final("GAME OVER");

    win_fi();		/* tanca les curses */

    elim_mem(id_camp);
    elim_mem(id_blocs);
    elim_mem(id_fi);
    elim_mem(id_pal);

    return (0);		/* retorna sense errors d'execucio */
}
