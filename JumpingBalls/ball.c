/*--------------------------------------------------------------*/
/*		SIMULATION OF JUMPING BALLS			*/
/*			WITH COLLISION                  */
/*--------------------------------------------------------------*/

#include "pmutex.h"
#include "ptask.h"
#include "tstat.h"
#include <allegro.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define ALLEGRO_NO_FIX_ALIASES /* fix for the new version of glibc */

#define XWIN      720                           /* screen width               */
#define YWIN      520                           /* screen height              */
#define PI        3.1415                        /* Pi                         */
#define BGC       0                             /* background color		      */
#define L         15                            /* dimension of a ball		  */
#define G         9.8                           /* acceleration of gravity    */
#define BASE      15                            /* position of the floor	  */
#define TOP       520                           /* initial height of the ball */
#define XMIN      15                            /* min position X of the ball */
#define XMAX      720                           /* max position Y of the ball */
#define VELX      10.                           /* horizontal ball velocity	  */
#define VMIN      9.                            /* minimum velocity           */
#define PER       20                            /* task period in ms		  */
#define DREL      20                            /* realtive deadline in ms	  */
#define PRIO      80                            /* ball task priority	      */
#define DPER      12                            /* period of draw_obstacle task*/
#define PRIO_O    81                            /* task priority              */
#define COLBROWN  makecol(139,69,19)            /* brown color                */
#define COLORG    makecol(127,255,0)            /* green color               */

float v0[MAX_TASKS];                            /* impact velocity with floor */
int x_coor[MAX_TASKS];                          /* x coordinate of the ball   */
int y_coor[MAX_TASKS];                          /* y coordinate of the ball   */

/* mutual exclusion semaphores  */
pthread_mutex_t mxa;                            /* = PTHREAD_MUTEX_INITIALIZER;*/
pthread_mutex_t mxv;                            /* = PTHREAD_MUTEX_INITIALIZER;*/
pthread_mutex_t mdraw;                          /* = PTHREAD_MUTEX_INITIALIZER;*/

struct obstacle
{
    int x;                                      /* obstacle's X location on map */
    int y;                                      /* obstacle's Y location on map */
    int up;                                     /* obstacle's up side           */
    int down;                                   /* obstacle's down side         */
    int right;                                  /* obstacle's right side        */
    int left;                                   /* obstacle's left side         */
    int visible;                                /* 0 visible  1  invisible      */
};

struct obstacle ob;
int end = 0;


/*--------------------------------------------------------------*/
/*              INIT FUNCTION           */
/*--------------------------------------------------------------*/
void init() {
    allegro_init();
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0); 
    clear_to_color(screen, BGC);
    install_keyboard();
    install_mouse();
    show_mouse(screen);
    srand(time(NULL));

    ob.visible    = 0;
    int SmallBoxX = XWIN / 2 ;
    int SmallBoxY = (YWIN / 2 + 150);
    int SmallW    = 200;
    int SmallH    = 140;
   

    rect(screen, XMIN + L + 15, BASE - 1, XMAX -40 + 1, TOP + BASE -40 + 1, COLORG );
    textout_centre_ex(screen, font, "SPACE to create a ball", XWIN / 2, YWIN / 2,
                      COLORG , BGC);
    textout_centre_ex(screen, font, "Click any where to create an obstacle", XWIN / 2, YWIN / 2 + 30,
                      COLORG , BGC);
    textout_centre_ex(screen, font, "ESC exit", XWIN / 2, YWIN / 2 + 60, COLORG , BGC);

    rect(screen, SmallBoxX -100, SmallBoxY + 70, SmallBoxX + 100, SmallBoxY - 70, COLORG );

    draw_ball(SmallBoxX, SmallBoxY, 4);
    draw_ball(SmallBoxX - 50, SmallBoxY + 20, 3);
    draw_ball(SmallBoxX + 80, SmallBoxY - 1, 5);

    pmux_create_pi(&mxa);
    pmux_create_pi(&mxv);
    pmux_create_pi(&mdraw);

    ptask_init(SCHED_FIFO, GLOBAL, PRIO_INHERITANCE);
        
}

/*--------------------------------------------------------------*/
/*              DRAW BALL FUNCTION             */
/*--------------------------------------------------------------*/

void draw_ball(int x, int y, int c) { circlefill(screen, x, y, L, c); }

/*--------------------------------------------------------------*/
/*              HANDLEING CREATE AN OBSTACLE         */
/*                   AND ITS MOVEMENT AND HAND       */
/*--------------------------------------------------------------*/

void draw_obstacle() {

    int x0, y0 ;                                /* rectangle's old position         */
    int x, y;                                   /* rectangle's new position         */
    int rx01, ry01, rx02, ry02;                 /* rectangle's old coordinates      */
    int rx1, ry1, rx2, ry2;                     /* rectangle coordinates points(1,2)*/
    int rx3, ry3, rx4, ry4;                     /* rectangle coordinates points(3,4)*/
    

    x0 = ob.x;
    y0 = ob.y;
    
    rx01 = x0;
    ry01 = y0;
    rx02 = x0 - 200 ;
    ry02 = y0 + 20; 

    x = mouse_x;
    y = mouse_y;

    rx1 = x ;
    ry1 = y ;
    rx2 = x - 200 ;  /* obtain the other point for the rectnagle */
    ry2 = y + 20;
    

  
    scare_mouse();
     /* if it is the first time to draw an obstcale,no need to erase the first one */
    if ((ob.visible = 0)){
        pthread_mutex_lock(&mdraw);
        rectfill(screen, rx1, ry1, rx2, ry2, COLBROWN);
        pthread_mutex_unlock(&mdraw);
    }
    else{
        pthread_mutex_lock(&mdraw);
        rectfill(screen, rx01, ry01, rx02, ry02, COLBROWN);
        rectfill(screen, rx01, ry01, rx02, ry02, BGC);
        rectfill(screen, rx1, ry1, rx2, ry2, COLBROWN);
        pthread_mutex_unlock(&mdraw);
    }
    
    show_mouse(screen);

    x0 = x;
    y0 = y;

    rx01 = rx1;
    ry01 = ry1;
    rx02 = rx2;
    ry02 = ry2;


    ob.x = x;
    ob.y = y;
    ob.visible = 1;
    ob.left = x - 200;
    ob.right = x;
    ob.up  = y + 20;
    ob.down = y;
    
    }



/*--------------------------------------------------------------*/
/*             	Collision Detection		*/
/*--------------------------------------------------------------*/
int collisionDetection (int i, int x1, int y1)
{
    //check for collission 
         for (int j = 0; j < MAX_TASKS; j++)
         {
             //get the distance between the two ball's centers 
            if(y_coor[j]!=0 && x_coor[j]!=0 && j!=i){
                float gdistance = ((x_coor[j]-x1)*(x_coor[j]-x1))+((y_coor[j]-y1)*(y_coor[j]-y1));
                gdistance = sqrt(gdistance);
                if(gdistance <= L)
                {
                    //there is a collision with ball j
                    // printf('there is a collision');
                    return j;
                    
                }
            }
            
         }
         // there is no collision 
         return -1;
}

/*--------------------------------------------------------------*/
/*	Periodic task for ball simulation			*/
/*--------------------------------------------------------------*/

void ball() {
    int i, col, dcol = 0; /* task index               */
    int x, y;             /* ball graphic coordinates */
    int ox, oy;           /* old ball position        */
    int x0;               /* starting position X ball */
    float vx, vy;         /* ball speed               */
    float t, tx;          /* temporary variable       */
    float dt;             /* time increment           */
    double a;             /* support variable         */  
    

    i = ptask_get_index();
    col = 4 + i % 14;
    y = oy = 20 + rand() % (SCREEN_W-L);  //y = y = oy = ( TOP - L);
    x = ox = x0 = 20 + rand() % (SCREEN_H-L);  //x = x = ox = x0 = XMIN;
    // save the initial coordinates of the ball 
    x_coor[i]=x;
    y_coor[i]=y;

    a = 2. * G * (float)(TOP - L); 
    vy = sqrt(a);
    vx = VELX;
    tx = 0.0;
    t = vy / G;
    dt = ptask_get_period(i, MILLI) / 100.;

    // removed, unless a specific activation from the main is done
  
    while (1) {

        //check collision with obstacle
        if(ob.visible ==1)
        {
         if (((x >= ob.left) && (x <= ob.right + L + 5 )) || ((x + L + 5 >= ob.left) && (x + L + 5 <= ob.right))){
            tx = 0.0;
            x0 = x;
            vx = -vx;
            x = x0 + vx * tx;
            x_coor[i]=x;

           }
        
        }

        int result = collisionDetection (i, x,y);
       
        if(result==-1)
            {
                x = x0 + vx * tx;
                y = BASE + vy * t - .5 * G * t * t;
                y_coor[i]=y;
                x_coor[i]=x;

                tx += dt;
            }
        else
        {
      
                // the ball at the left of the collied object
                if(x < x_coor[result])
                { 
                    tx = 0.4;
                    x0 = L-x_coor[result];
                    vx =  vx - 1 % 10;                  
                    x = x0 + vx * tx;
                    x_coor[i]=x;
                }
                // the ball at the right of the collied object
                else 
                {
                    tx = 0.0;
                    x0 = L+x_coor[result];
                    vx = -vx; 
                    x = x0 + vx * tx;
                    x_coor[i]=x;
                }
            
        }
        
        if (y < BASE) {
            t = 0.0;
            pthread_mutex_lock(&mxv);
            v0[i] = .9 * v0[i];
            vy = v0[i];
            pthread_mutex_unlock(&mxv);
            y = BASE + vy * t - .5 * G * t * t;
            y_coor[i]=y;
        }
        if (x > XMAX) {
            tx = 0.0;
            x0 = XMAX;
            vx = -vx;
            x = x0 + vx * tx;
            x_coor[i]=x;
        }
        if (x < XMIN) {
            tx = 0.0;
            x0 = XMIN;
            vx = -vx;
            x = x0 + vx * tx;
            x_coor[i]=x;
        }
    
        pthread_mutex_lock(&mxa);
        draw_ball(ox, YWIN - oy, BGC);
        draw_ball(x, YWIN - y, col);
        pthread_mutex_unlock(&mxa);

        oy = y;
        ox = x;
        t += dt;
        tx += dt;

        /* check for deadline miss */
        if (ptask_deadline_miss()) {
            dcol = dcol % 15 + 1;
            pthread_mutex_lock(&mxa);
            rectfill(screen, 400, 50, 450, 70, dcol);
            pthread_mutex_unlock(&mxa);
        }
        ptask_wait_for_period();
    }
}

/*--------------------------------------------------------------*/
/*			MAIN process				*/
/*--------------------------------------------------------------*/

int main(void) {
    int c;                              /* character from keyboard   	*/
    int i, j, k, o;                        /* number of tasks created	    */
    double a;                           /* temporary variable           */
    int h;                              /* temporary variable           */
    int ntasks = 0;                     /* total number of created tasks*/
    int last_proc = 0;                  /* last assigned processor      */
    int max_proc = ptask_getnumcores(); /* max number of procs          */
   

    init();
    
    a = 2. * G * (float)TOP;
    for (i = 0; i < MAX_TASKS; i++)
        {
            v0[i] = sqrt(a);
            x_coor[i]= 0;
            y_coor[i]=0;
        }


    i = 0;
    do {
        k = 0;
        if (keypressed()) {
            c = readkey();
            k = c >> 8;
        }
        if ((ntasks == 0) && (k == KEY_SPACE)) {
            clear_to_color(screen, BGC);
             
        }
        if ((ntasks == 0) && (k == KEY_SPACE)) {
            clear_to_color(screen, BGC);
          
        }
        if (((ntasks == 0) || (ntasks < MAX_TASKS)) && (mouse_b & 1)){
            tpars params1 = TASK_SPEC_DFL;
            params1.period = tspec_from(DPER, MILLI);
            params1.rdline = tspec_from(DREL, MILLI);
            params1.priority = PRIO - i;
            params1.measure_flag = 1;
            params1.act_flag = NOW;
            /* a round robin assignment */
            params1.processor = last_proc++;
            if (last_proc >= max_proc)
                last_proc = 0;
            o = ptask_create_param(draw_obstacle, &params1);
            
            printf("Task %d created and activated\n", o);
        

        }

        if ((ntasks < MAX_TASKS) && (k == KEY_SPACE)) {
            tpars params = TASK_SPEC_DFL;
            params.period = tspec_from(PER, MILLI);
            params.rdline = tspec_from(DREL, MILLI);
            params.priority = PRIO - i;
            params.measure_flag = 1;
            params.act_flag = NOW;
            /* a round robin assignment */
            params.processor = last_proc++;
            if (last_proc >= max_proc)
                last_proc = 0;

            
            i = ptask_create_param(ball, &params);
            
            if (i != -1) {
                printf("Task %d created and activated\n", i);
                ntasks++;
            } else {
                allegro_exit();
                printf("Error in creating task!\n");
                exit(-1);
            }

        }

        if ((k >= KEY_0) && (k <= KEY_9)) {
            a = 2. * G * (float)TOP;
            pthread_mutex_lock(&mxv);
            v0[k - KEY_0] = sqrt(a);
            pthread_mutex_unlock(&mxv);
        }

        if ((k == KEY_O) && (ntasks > 9)) {
            for (j = 10; j < ntasks; j++) {
                h = rand() % (TOP - BASE);
                a = 2. * G * (float)h;
                pthread_mutex_lock(&mxv);
                v0[j] = sqrt(a);
                pthread_mutex_unlock(&mxv);
            }
        }

        if (k == KEY_A) {
            for (j = 0; j < ntasks; j++) {
                h = rand() % (TOP - BASE);
                a = 2. * G * (float)h;
                pthread_mutex_lock(&mxv);
                v0[j] = sqrt(a);
                pthread_mutex_unlock(&mxv);
            }
        }

    } while (k != KEY_ESC);

    printf("Now printing the stats\n");
    for (j = 0; j < ntasks; j++) {
        tspec wcet = ptask_get_wcet(j);
        tspec acet = ptask_get_avg(j);

        printf("TASK %d: WCET = %ld\t ACET = %ld\t NINST=%d\n", j,
               tspec_to(&wcet, MICRO), tspec_to(&acet, MICRO),
               ptask_get_numinstances(j));
    }

    printf("End of statistics\n");
    allegro_exit();
    return 0;
}

/*--------------------------------------------------------------*/
