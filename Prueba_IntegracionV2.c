/*
 * Prueba de integración funciones de proteccion + LIBIEC61850
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include "goose_receiver.h"
#include "goose_subscriber.h"
#include "linked_list.h"
#include "mms_value.h"
#include "goose_publisher.h"
#include "hal_thread.h"

static int running=0;

void sigint_handler(int signalId)
{
        running=0;
}

// Functions for LinkedList Update

LinkedList updateDataSetValues(float Ts) {
    // Replace this logic with your desired dataset value generation
    LinkedList dataSetValues = LinkedList_create();
    LinkedList_add(dataSetValues, MmsValue_newFloat(Ts));
    return dataSetValues;
}


//Variables updated online

static int yop[3];  // Mode of operation neighbor devices
static int top_ext[3]; //Operating time neighbor devices
static int blk; //Blocking Signal
static float V_meas; //Reporting of possitive sequence  voltage magnitude
static float I_meas; //Reporting of possitive sequence current magnitude
static float V_ang;  // Reporting of possitive sequence voltage angle
static float I_ang;  // Reporting of possitive sequence current angle
static float Ang_res; //Calculated restraint angle
static float I_res;  //Calculated restraint current
static int Rloc=0; //Local operating mode
static int top=100000;
static  int AG=3; /* Agent type 1(generator) 2(load) 3(line)*/
static  float CTI=300;
static int blkl=3; //Transmited blocking signal
//Functions for fault detection

void Calc_ang() {
  float Vth=0.99;
  if(V_meas>=Vth){
    Ang_res=V_ang-I_ang;
    I_res=I_meas;
  }
}

void Prot_Carga() {
  float Vth=0.99;
  float Ith=I_res*1.2;
  float Th_rf=V_ang-I_ang;

  if((I_meas>Ith && V_meas<Vth)||(V_meas<Vth && I_res-I_meas<0)){
    if(fabs(Th_rf-Ang_res)<5){
      Rloc=0;
    }
    else {
      if(Th_rf>=90 || Th_rf<=-90){
        Rloc=-1;
      }
      else{
        Rloc=1;
      }
    }
  }
  else {
    Rloc=0;
  }
}


// Listener function to receive and update information in synchronous mode


static void
gooseListener(GooseSubscriber subscriber, void* parameter)
{
 printf("GOOSE event:\n");
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);
    char buffer[1024];
    MmsValue_printToBuffer(values, buffer, 1024);
    const char* delimiter = "{},";
    char* token = strtok(buffer, delimiter);
    int extp[8];
    int count =0;
    while (token != NULL) {
        int val= atoi(token);  // Convert token to integer
        extp[count++]=val;
        token = strtok(NULL, delimiter);
    }
    yop[0]=extp[0];
    yop[1]=extp[1];
    yop[2]=extp[2];
    yop[3]=extp[3];
    top_ext[0]=extp[4];
    top_ext[1]=extp[5];
    top_ext[2]=extp[6];
    top_ext[3]=extp[7];
    blk=extp[8];
    printf("Prueba: %f\n",blk);
}


static void
gooseListener2(GooseSubscriber subscriber, void* parameter)
{
 printf("GOOSE event:\n");
    MmsValue* values2 = GooseSubscriber_getDataSetValues(subscriber);
    char buffer2[1024];
    MmsValue_printToBuffer(values2, buffer2, 1024);
    const char* delimiter2 = "{},";
    char* token2 = strtok(buffer2, delimiter2);
    float extp2[3];
    int count2 =0;
    while (token2 != NULL) {
        float val2= atoi(token2);  // Convert token to integer
        extp2[count2++]=val2;
        token2 = strtok(NULL, delimiter2);
    }

    V_meas=extp2[0];
    I_meas=extp2[1];
    V_ang=extp2[2];
    I_ang=extp2[3];
    Calc_ang();
    printf("Prueba: %f\n",Ang_res);
}


//  Functions for protection coordination

void OptimDistribuida(){

  int p=0;
  if ((AG == 1 || AG == 2) && Rloc != 0){ /*Load and generator coordination*/
    if (Rloc ==1){
      top=10;   /* Primary operation*/
    }
    else if (Rloc == -1){
      for (int i=0;i<4;i++){
          if(yop[i] == 1){
            p=i;
          }
      }
      int h=2;
      while (h != 1){
        top=top_ext[p]+310;  /* Backup operation*/
        if (top_ext[p]-top <= -CTI){
          h=1;
        }
      }
    }
  }
  else if (AG ==3 && Rloc != 0){ /* Line agent coordination*/
      if (Rloc == 1){
        blkl=1; /* Block signal for primary oper*/
        if (blk==1){
          top = 10; /* Primary operation*/
        }
        else if(blk==0){
          top=350; /* Remote backup operation*/
        }
        else{
          top=10000000; /* Block operation*/
        }
      }
      else if (Rloc == -1){  /* Backup mode */
        int t=0;
        for (int i=0;i<4;i++){
          printf("%d\n",yop[i] );
          if (yop[i] == 1){
            t=t+1;
          }
        }
        if (t==0){
          top=10; /* Primary operation*/
          blkl=0;   /* Block signal for remote backup*/
        }
        else{
          blkl=2; /* Block signal for blocking */

          for (int i=0;i<4;i++){
              if(yop[i] == 1){
                p=i;
              }
            }

          int h=2;
          while (h != 1){
            top=top_ext[p]+310;  /* Local backup operation */
            if (top_ext[p]-top <= -CTI){
              h=1;
            }
          }
        }
      }
  }
}
/* has to be executed as root! */
int
main(int argc, char **argv)
{
    char *interface;

    if (argc > 1)
        interface = argv[1];
    else
        interface = "eth0";

    printf("Using interface %s\n", interface);


// Creación del receptor de GOOSE y el subscriptor1

        GooseReceiver receiver = GooseReceiver_create();
        GooseReceiver_setInterfaceId(receiver,"eth0");
        GooseSubscriber subscriber = GooseSubscriber_create("PP1/Application/LLN0/Control_DataSet",NULL);
        uint8_t dstMac[6] = {0x01,0x0c,0xcd,0x01,0x00,0x00};
        GooseSubscriber_setDstMac(subscriber, dstMac);
        GooseSubscriber_setAppId(subscriber, 0x0001);
        GooseReceiver_addSubscriber(receiver,subscriber);
        GooseReceiver_start(receiver);

// Creación del receptor de GOOSE y el subscriptor2

        GooseReceiver receiver2 = GooseReceiver_create();
        GooseReceiver_setInterfaceId(receiver2,"eth0");
        GooseSubscriber subscriber2 = GooseSubscriber_create("SIP7Ln1_27Undervoltage1/LLN0/Control_DataSet",NULL);
        uint8_t dstMac2[6] = {0x01,0x0c,0xcd,0x01,0x00,0x01};
        GooseSubscriber_setDstMac(subscriber2, dstMac2);
        GooseSubscriber_setAppId(subscriber2, 0x0002);
        GooseReceiver_addSubscriber(receiver2,subscriber2);
        GooseReceiver_start(receiver2);


// Parámetros de comunicación

    CommParameters gooseCommParameters;
// MAC goose1
    gooseCommParameters.appId = 0x0003;
    gooseCommParameters.dstAddress[0] = 0x01;
    gooseCommParameters.dstAddress[1] = 0x0c;
    gooseCommParameters.dstAddress[2] = 0xcd;
    gooseCommParameters.dstAddress[3] = 0x01;
    gooseCommParameters.dstAddress[4] = 0x00;
    gooseCommParameters.dstAddress[5] = 0x02;
    gooseCommParameters.vlanId = 10;
    gooseCommParameters.vlanPriority = 4;

    /*
     * Create a new GOOSE publisher instance. As the second parameter the interface
     * name can be provided (e.g. "eth0" on a Linux system). If the second parameter
     * is NULL the interface name as defined with CONFIG_ETHERNET_INTERFACE_ID in
     * stack_config.h is used.
     */
    GoosePublisher publisher = GoosePublisher_create(&gooseCommParameters, interface);

    if (publisher) {
        /* Creación de goose control blocks*/

        GoosePublisher_setGoCbRef(publisher, "Rele1LDevice/LLN0$GO$Delay1");
        GoosePublisher_setConfRev(publisher, 30001);
        GoosePublisher_setGoID(publisher,"Delay1");
        GoosePublisher_setDataSetRef(publisher, "Rele1LDevice/LLN0$Delay1");
        GoosePublisher_setTimeAllowedToLive(publisher, 6000);

         }
    else {
        printf("Failed to create GOOSE publisher. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
        GoosePublisher_destroy(publisher);
        return 1;
    }

        int count=0;
        GooseSubscriber_setListener(subscriber,gooseListener,NULL);
        GooseSubscriber_setListener(subscriber2,gooseListener2,NULL);



//Main Loop
        running=1;
        signal(SIGINT,sigint_handler);
while(running){
//Actualización de los datasets
        Prot_Carga();

        OptimDistribuida();
        if (Rloc !=0){
          GoosePublisher_increaseStNum(publisher);
        }
        LinkedList dataSetValues=updateDataSetValues(top);

//Publicación de Datasets

        GoosePublisher_publish(publisher,dataSetValues);

        Thread_sleep(1);
        LinkedList_destroyDeep(dataSetValues,(LinkedListValueDeleteFunction) MmsValue_delete);

}

//Destrucción  de GoCB
      GoosePublisher_destroy(publisher);
      GooseReceiver_stop(receiver);
      GooseReceiver_destroy(receiver);
      GooseReceiver_stop(receiver2);
      GooseReceiver_destroy(receiver2);
    return 0;
}
