#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<pthread.h>
#include <unistd.h>
#include <sys/wait.h>

/* Include libsoc header files for pwm and file descriptor */
#include "libsoc_pwm.h"
#include "libsoc_debug.h"


static pthread_t adc;
static pthread_t servo1;
static pthread_t servo2;
static int vertical_flag,horizontal_flag; /*The threads read this flag and make decisions based on it */

/*Function that sets vertical and horizontal flag depending upon which conditions become true */
int set_flag(int lt,int rt,int ld,int rd){
    int avt = (lt + rt) / 2; // average value top
    int avd = (ld + rd) / 2; // average value down
    int avl = (lt + ld) / 2; // average value left
    int avr = (rt + rd) / 2; // average value right

    printf("Average top: %d\n",avt);
    printf("Average down: %d\n",avd);
    printf("Average left: %d\n",avl);
    printf("Average right: %d\n",avr);
    int dvert=avt-avd;
    int dhoriz=avl-avr;
    int tol=20;

    /* If outside tolerance */
    if(-1*tol>dvert || dvert>tol ){

        /* If bottom is receiving more light than top,
        implies rotate the vertical servo in clockwise direction.
        In order to do so, decrease the angle or increase the duty cycle.
        Finally set the vertical flag to -1 */

        if(avt>avd){
            vertical_flag=-1;
        }
        else if(avt<avd){
            vertical_flag=1;
        }

    }
        /* If within tolerance do nothing */
    else{
        vertical_flag=0;
    }

    if(-1*tol>dhoriz || dhoriz>tol ){

        /* If right is receiving more light than left,
        implies rotate the horizontal servo in clockwise direction.
        In order to do so, increase the angle or decrease the duty cycle.
        Finally, set the horizontal flag to 1*/

        if(avl>avr){
            horizontal_flag=1;
        }
        else if(avl<avr){
            horizontal_flag=-1;
        }
    }

    else{
        horizontal_flag=0;
    }
    return 0;

}

/*Read thread that reads the input from the sensors and passes it to set_flag. Reads every 1 second */
static void *adc_read(void *arg){
    FILE * tl;
    FILE * tr;
    FILE * dl;
    FILE * dr;
/*Declaring buffer to hold output from different files */
    char* topl = (char*)malloc(100);
    char* topr = (char*)malloc(100);
    char* downl = (char*)malloc(100);
    char* downr = (char*)malloc(100);

/*Opens, copies and closes adc files sleeping for 1 second every iteration. Continues iterating indefinitely */
    while(1){
        tl=fopen("/sys/bus/iio/devices/iio:device0/in_voltage2_raw","r");
        fgets(topl,100,tl);
        printf("Top left: %s\n",topl);
        int valtl=atoi(topl);
        printf("Int top left: %d\n",valtl);
        fclose(tl);


        tr=fopen("/sys/bus/iio/devices/iio:device0/in_voltage3_raw","r");
        fgets(topr,100,tr);
        int valtr=atoi(topr);
        printf("Top right: %s\n",topr);
        fclose(tr);


        dl=fopen("/sys/bus/iio/devices/iio:device0/in_voltage0_raw","r");
        fgets(downl,100,dl);
        int valdl=atoi(downl);
        printf("Down left: %s\n",downl);
        fclose(dl);

        dr=fopen("/sys/bus/iio/devices/iio:device0/in_voltage1_raw","r");
        fgets(downr,100,dr);
        int valdr=atoi(downr);
        printf("Down right: %s\n",downr);
        fclose(dr);

        set_flag(valtl,valtr,valdl,valdr);
        sleep(1);
    }
    free(topl);
    free(topr);
    free(downl);
    free(downr);
    return (void *)0;
}

/* Thread to control the horizontal servo */
static void *horizontal(void *arg1)
{


    float SRV_0  =  0.45; //Setting the servo value for angle 0
/*  */
    float SRV_180  =2.45; //Setting the servo value for angle 180

/*  */
    float FRQ =50.0; //Choosing frequency of the PWM to be 50
/* */
    float PER =20.0;


    int ret = EXIT_SUCCESS;

    libsoc_set_debug(1); //Prints out the file description. Used to debug errors associated with libsoc

    pwm *pwmh = libsoc_pwm_request(3, 0, LS_PWM_SHARED); /* A pointer of pwm struct type holds the return value of request function.
														This function makes the request to enable pwmchip3 on shared mode LS_PWM_SHARED.
														In this mode the pwm will not be unexported, if it is exported on freeing pwm. */


    if (!pwmh)
    {
        printf("Failed to get PWM\n");
        goto fail;
    }

    int period = PER * 1.0E6;	/* */
    libsoc_pwm_set_period(pwmh, period); /* Used to set the period of PWM for the servo */

    int current_period = libsoc_pwm_get_period(pwmh); /* Used to retrieve the PWM period that has been just set for the servo */
    if (current_period != period){
        printf("Failed period setting\n");
        goto fail;
    }

    int i=90; /* Intitializing the servo to a 90 degree position */
    float SM_1_duty ;	/*  */
    int enabled;


    /*  */
    /*  */
    SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0; /* Used to compute duty percentage*/
    printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

    int duty = (SM_1_duty * 1.0E-2) * period; /* calculate active time of the duty */

    libsoc_pwm_set_duty_cycle(pwmh, duty);	/* sets the duty cycle of the pwm pulse based on the active pulse sent to it */

    int current_duty = libsoc_pwm_get_duty_cycle(pwmh);
    if (current_duty != duty) {
        printf("Failed duty test\n");
        goto fail;
    }

    libsoc_pwm_set_enabled(pwmh, ENABLED);	/* This sets the state of the servo to enabled telling it to wait for input*/
    enabled = libsoc_pwm_get_enabled(pwmh);	/* */
    if (!enabled) {
        printf("Failed enabling test\n");
        ret = EXIT_FAILURE;
        goto fail;
    }

    sleep(2); /* */

    libsoc_pwm_set_enabled(pwmh, DISABLED);	/* This is used to set the state of the servo to disabled */
    enabled = libsoc_pwm_get_enabled(pwmh);	/*  */
    if (enabled) {
        printf("Failed disabling test\n");
        ret = EXIT_FAILURE;
        goto fail;
    }



/* Iterate as long as the servo angle is between 0 to 180 */
    while(i<=180 && i>0){

        /* If the horizontal flag is set to 1, decrease the duty cycle. This increases the servo rotation angle */
        if(horizontal_flag==1){

            i+=10; /* Increment of 10 */
            if(i>180){
                i=180;
            }
            SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
            printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

            int duty = (SM_1_duty * 1.0E-2) * period; /*  */

            libsoc_pwm_set_duty_cycle(pwmh, duty);	/*  */

            int current_duty = libsoc_pwm_get_duty_cycle(pwmh);
            if (current_duty != duty) {
                printf("Failed duty test\n");
                goto fail;
            }

            libsoc_pwm_set_enabled(pwmh, ENABLED);	/* */
            enabled = libsoc_pwm_get_enabled(pwmh);	/* */
            if (!enabled) {
                printf("Failed enabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }

            sleep(2); /* */

            libsoc_pwm_set_enabled(pwmh, DISABLED);	/*  */
            enabled = libsoc_pwm_get_enabled(pwmh);	/*  */
            if (enabled) {
                printf("Failed disabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }
        }
            /* If the horizontal flag is set to -1, increase the duty cycle. This decreases the servo rotation angle */
        else if(horizontal_flag==-1){
            i-=10; /* Decrement of 10 */
            if(i<0){
                i=0;
            }
            SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
            printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

            int duty = (SM_1_duty * 1.0E-2) * period; /*  */

            libsoc_pwm_set_duty_cycle(pwmh, duty);	/*  */

            int current_duty = libsoc_pwm_get_duty_cycle(pwmh);
            if (current_duty != duty) {
                printf("Failed duty test\n");
                goto fail;
            }

            libsoc_pwm_set_enabled(pwmh, ENABLED);	/* */
            enabled = libsoc_pwm_get_enabled(pwmh);	/* */
            if (!enabled) {
                printf("Failed enabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }

            sleep(2); /* */

            libsoc_pwm_set_enabled(pwmh, DISABLED);	/*  */
            enabled = libsoc_pwm_get_enabled(pwmh);	/*  */
            if (enabled) {
                printf("Failed disabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }
        }
    }
    fail:

    libsoc_pwm_free(pwmh);

    return (void *)123;

}

/* The thread that controls the rotation of vertical servo.
The way this thread controls the rotation is same as that of the horizontal servo
except that it reads from the vertical servo flag */

static void *vertical(void *arg2){

    float SRV_0  =  0.45;
    /*  */
    float SRV_180  =2.45;

    /*  */
    //float FRQ =50.0;
    /* */
    float PER =20.0;


    int ret = EXIT_SUCCESS;

    libsoc_set_debug(1);

    pwm *pwm = libsoc_pwm_request(6, 0, LS_PWM_SHARED); /* The request is made to pwmchip6 */

    if (!pwm)
    {
        printf("Failed to get PWM\n");
        goto fail;
    }

    int period = PER * 1.0E6;	/* */
    libsoc_pwm_set_period(pwm, period);

    int current_period = libsoc_pwm_get_period(pwm);
    if (current_period != period){
        printf("Failed period setting\n");
        goto fail;
    }

    int i=90; /* Intitializing the vertical servo to a 90 degree position */
    float SM_1_duty ;	/*  */
    int enabled;


    /*  */
    /*  */
    SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
    printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

    int duty = (SM_1_duty * 1.0E-2) * period; /* Calculate duty */

    libsoc_pwm_set_duty_cycle(pwm, duty);	/* Set duty */

    int current_duty = libsoc_pwm_get_duty_cycle(pwm);
    if (current_duty != duty) {
        printf("Failed duty test\n");
        goto fail;
    }

    libsoc_pwm_set_enabled(pwm, ENABLED);	/* */
    enabled = libsoc_pwm_get_enabled(pwm);	/* */
    if (!enabled) {
        printf("Failed enabling test\n");
        ret = EXIT_FAILURE;
        goto fail;
    }

    sleep(2); /* */

    libsoc_pwm_set_enabled(pwm, DISABLED);	/*  */
    enabled = libsoc_pwm_get_enabled(pwm);	/*  */
    if (enabled) {
        printf("Failed disabling test\n");
        ret = EXIT_FAILURE;
        goto fail;
    }

    /*Iterate until the servo angle is between 0 and 180 */

    while(i<=180 && i>0){
        if(vertical_flag==1){

            i+=10;
            if(i>180){
                i=180;
            }
            SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
            printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

            int duty = (SM_1_duty * 1.0E-2) * period; /*  */

            libsoc_pwm_set_duty_cycle(pwm, duty);	/*  */

            int current_duty = libsoc_pwm_get_duty_cycle(pwm);
            if (current_duty != duty) {
                printf("Failed duty test\n");
                goto fail;
            }

            libsoc_pwm_set_enabled(pwm, ENABLED);	/* */
            enabled = libsoc_pwm_get_enabled(pwm);	/* */
            if (!enabled) {
                printf("Failed enabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }

            sleep(2); /* */

            libsoc_pwm_set_enabled(pwm, DISABLED);	/*  */
            enabled = libsoc_pwm_get_enabled(pwm);	/*  */
            if (enabled) {
                printf("Failed disabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }
        }

        else if(vertical_flag==-1){
            i-=10;
            if(i<0){
                i=0;
            }
            SM_1_duty = 100.0 - ((SRV_0/PER) + (i/180.0) * ((SRV_180-SRV_0)/PER))*100.0;
            printf("Angle : %d , duty : %f %%\n" ,i ,SM_1_duty);

            int duty = (SM_1_duty * 1.0E-2) * period; /*  */

            libsoc_pwm_set_duty_cycle(pwm, duty);	/*  */

            int current_duty = libsoc_pwm_get_duty_cycle(pwm);
            if (current_duty != duty) {
                printf("Failed duty test\n");
                goto fail;
            }

            libsoc_pwm_set_enabled(pwm, ENABLED);	/* */
            enabled = libsoc_pwm_get_enabled(pwm);	/* */
            if (!enabled) {
                printf("Failed enabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }

            sleep(2); /* */

            libsoc_pwm_set_enabled(pwm, DISABLED);	/*  */
            enabled = libsoc_pwm_get_enabled(pwm);	/*  */
            if (enabled) {
                printf("Failed disabling test\n");
                ret = EXIT_FAILURE;
                goto fail;
            }
        }
    }
    fail:
    libsoc_pwm_free(pwm);
    return (void *)112;
}

int main()
{
    void *returnvalue;
    int parameter=0;

    /*The main program creates three threads and assigns each of them a different function to execute */

    pthread_create(&adc,NULL,adc_read,(void *)parameter);
    pthread_create(&servo1,NULL,horizontal,(void *)parameter);
    pthread_create(&servo2,NULL,vertical,(void *)parameter);

    /* waits for each thread to join with it and eventually terminate the program */

    pthread_join(adc,&returnvalue);
    pthread_join(servo1,&returnvalue);
    pthread_join(servo2,&returnvalue);
    return 0;

}