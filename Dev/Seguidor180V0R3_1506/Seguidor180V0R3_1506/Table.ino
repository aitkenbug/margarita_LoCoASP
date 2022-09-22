//THIS CODE TURNS THE MONTH INTO THE NUMBER OF DAYS SINCE JANUARY 1ST.
//ITS ONLY PURPOSE IS FOR CALCULATING DELTA (DECLINATION), AND IS NOT USED IN THE HOUR ANGLE TABLE OR ANYWHERE ELSE.
      /*int daynum(int month){
       int data;
       switch (month){
        case 1:data=0;break;
        case 2:data=31;break;
        case 3:data=59; break;
        case 4:data=90;break;
        case 5:data=120;break;
        case 6:data=151;break;
        case 7:data=181;break;
        case 8:data=212;break;
        case 9:data=243;break;
        case 10:data=273;break;
        case 11:data=304;break;
        case 12:data=334;break;
        return data; 
      }
        
        
        
        
        
        }*/
 
       

//THIS CODE TAKES THE DAY OF THE MONTH AND DOES ONE OF THREE THINGS: ADDS A DAY, SUBTRACTS A DAY, OR
//DOES NOTHING. THIS IS DONE SO THAT LESS VALUES ARE REQUIRED FOR THE NOON HOUR ANGLE TABLE BELOW.
       int dayToArrayNum(int day){
        return ((day-1)/3)-(day/31);
        
        /*
         * son matematicamente equivalentes, asumiendo que day vive entre 1 y 31
         * lo dejo por si resulta no ser equivalente
        switch(day){
          case 1:
          case 2:
          case 3:
          day=0;
          break;
          case 4:
          case 5:
          case 6:
          day=1;
          break;
          case 7:
          case 8:
          case 9:
          day=2;
          break;
          case 10:
          case 11:
          case 12:
          day=3;
          break;
          case 13:
          case 14:
          case 15:
          day=4;
          break;
          case 16:
          case 17:
          case 18:
          day=5;
          break;
          case 19:
          case 20:
          case 21:
          day=6;
          break;
          case 22:
          case 23:
          case 24:
          day=7;
          break;
          case 25:
          case 26:
          case 27:
          day=8;
          break;
          case 28:
          case 29:
          case 30:
          case 31:
          day=9;
          break;
          
          }
          return day;
          */
       }
//////////////////////////////////////////////////////////////
//HERE IS THE TABLE OF NOON HOUR ANGLE VALUES. THESE VALUES GIVE THE HOUR ANGLE, IN DEGREES, OF THE SUN AT NOON (NOT SOLAR NOON)
//WHERE LONGITUDE = 0. DAYS ARE SKIPPED TO SAVE SPACE, WHICH IS WHY THERE ARE NOT 365 NUMBERS IN THIS TABLE.
const float tabla[12] [10]={
       {-1.038,-1.379,-1.703,-2.007,-2.289,-2.546,-2.776,-2.978,-3.151,-3.294,},
       {-3.437,-3.508,-3.55,-3.561,-3.545,-3.501,-3.43,-3.336,-3.219,-3.081,},
       {-2.924,-2.751,-2.563,-2.363,-2.153,-1.936,-1.713,-1.487,-1.26,-1.035,},
       {-0.74,-0.527,-0.322,-0.127,0.055,0.224,0.376,0.512,0.63,0.728,},
       {0.806,0.863,0.898,0.913,0.906,0.878,0.829,0.761,0.675,0.571,},
       { 0.41,0.275,0.128,-0.026,-0.186,-0.349,-0.512,-0.673,-0.829,-0.977,},
       {-1.159,-1.281,-1.387,-1.477,-1.547,-1.598,-1.628,-1.636,-1.622,-1.585,},
       {-1.525,-1.442,-1.338,-1.212,-1.065,-0.9,-0.716,-0.515,-0.299,-0.07,},
       {0.253,0.506,0.766,1.03,1.298,1.565,1.831,2.092,2.347,2.593,},
       { 2.828,3.05,3.256,3.444,3.613,3.759,3.882,3.979,4.049,4.091,},
       {4.1,4.071,4.01,3.918,3.794,3.638,3.452,3.236,2.992,2.722,},
       {2.325,2.004,1.665,1.312,0.948,0.578,0.205,-0.167,-0.534,-0.893,}
       };
       
float FindH(int day, int month){
 //es como una tabla ineficiente, siento que en vez de programa debiera ser guardado como un array 2D en caso de que falte memoria aca se corta
      return tabla[month-1][day]; //Espero que la tabla haya sido construido como dia 0 = primer dia del mes
      /*
      float h;
      
      switch(month){
        case 1:{ float h_Array[10]={-1.038,-1.379,-1.703,-2.007,-2.289,-2.546,-2.776,-2.978,-3.151,-3.294,};h = h_Array[day];break;}
        case 2:{float h_Array[10]={-3.437,-3.508,-3.55,-3.561,-3.545,-3.501,-3.43,-3.336,-3.219,-3.081,};h = h_Array[day];break;}
        case 3:{float h_Array[10]={-2.924,-2.751,-2.563,-2.363,-2.153,-1.936,-1.713,-1.487,-1.26,-1.035,};h = h_Array[day];break;}
        case 4:{float h_Array[10]={-0.74,-0.527,-0.322,-0.127,0.055,0.224,0.376,0.512,0.63,0.728,};h = h_Array[day];break;}
        case 5:{float h_Array[10]={0.806,0.863,0.898,0.913,0.906,0.878,0.829,0.761,0.675,0.571,};h = h_Array[day];break;}
        case 6:{float h_Array[10]={ 0.41,0.275,0.128,-0.026,-0.186,-0.349,-0.512,-0.673,-0.829,-0.977,};h = h_Array[day];break;}
        case 7:{float h_Array[10]={-1.159,-1.281,-1.387,-1.477,-1.547,-1.598,-1.628,-1.636,-1.622,-1.585,};h = h_Array[day];break;}
        case 8:{float h_Array[10]={-1.525,-1.442,-1.338,-1.212,-1.065,-0.9,-0.716,-0.515,-0.299,-0.07,};h = h_Array[day];break;}
        case 9:{float h_Array[10]={0.253,0.506,0.766,1.03,1.298,1.565,1.831,2.092,2.347,2.593,};h = h_Array[day];break;}
        case 10:{float h_Array[10]={ 2.828,3.05,3.256,3.444,3.613,3.759,3.882,3.979,4.049,4.091,};h = h_Array[day];break;}
        case 11:{float h_Array[10]={ 4.1,4.071,4.01,3.918,3.794,3.638,3.452,3.236,2.992,2.722,};h = h_Array[day];break;}
        case 12:{float h_Array[10]={2.325,2.004,1.665,1.312,0.948,0.578,0.205,-0.167,-0.534,-0.893,};h = h_Array[day];break;}
      }
      
      return h;*/
      };
        
//////////////////////////////////////////////////////////////
