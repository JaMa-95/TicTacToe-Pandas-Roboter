#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


#include <sstream>
#include <termios.h>

int player = 1;
int msg_valid = 1;

//Eistellungen für Tastatureingabe (Jede Taste besitzt eine Zahl)
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new setting
  int ch = getchar();                        // read character (non-blocking
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return ch;
}

//Callback ob vorheriger Zug gueltig ist
void checkGameCallback(const std_msgs::Int8::ConstPtr& msg_fault)

{
	int msg_data = msg_fault->data;
	//Auswertung der Nachricht von eval_game
	if (msg_data == 1)
	{
   	//ROS_INFO("Spielzug: [OK]");
    	msg_valid = 1;
	}

	//Spielzug ist ungültig Spieler muss nochmal ein gültiges Feld auswählen
	if (msg_data == 2)
	{
   	//ROS_INFO("Spielzug: [Ungueltig]");
		if (player == 1)
		{
    		player = player +1;
		}
		else
		{
    		player = player -1;
		}

  		msg_valid = 1;
	}

	//Spieler 1 gewinnt
	if (msg_data == 3)
	{
   	ROS_INFO("Es gewinnt Spieler 1!");
    ROS_INFO("Das Spiel ist beendet. Sie koennen das Programm ueber Ctrl-C schliessen.");
    msg_valid = -1;
	}

	//Spieler 2 gewinnt
	if (msg_data == 4)
	{
   	ROS_INFO("Es gewinnt Spieler 2!");
    ROS_INFO("Das Spiel ist beendet. Sie koennen das Programm ueber Ctrl-C schliessen.");
    msg_valid = -1;
	}

  	//Unetschieden
	if (msg_data == 5)
	{
   	ROS_INFO("Es gibt ein Unentschieden!");
    ROS_INFO("Das Spiel ist beendet. Sie koennen das Programm ueber Ctrl-C schliessen.");
    msg_valid = -1;
	}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "communication_player");
  ros::NodeHandle nc;
  ros::Publisher pub_next_move = nc.advertise<std_msgs::Int8MultiArray>("next_move", 100);
  ros::Subscriber sub_end_eval_game = nc.subscribe("end_eval_game", 100, checkGameCallback);
  ros::Rate loop_rate(10);

  ros::WallDuration(4).sleep();

  int count = 0;

  while (ros::ok())
  {
	if (msg_valid == 1)
	{
	std_msgs::Int8MultiArray next_move;

	ROS_INFO("Spieler %d bitte Spielfeldnummer eingeben", player);
	int  c = getch();  // Tastatureingabe wird erfordert
	c = c-48;
	ROS_INFO("%d",c);
	//!!!!ACHTUNG!!!! FALLS TASTATUR ZWEI NUMMERNFELDER BESITZT, FUNKTIONIERT MEIST NUR EINE KORREKT

	while ((c > 10) or ( c <= 0)) // Zahl darf nicht 0 oder größer als 9 sein
	{
	ROS_INFO("Zahl muss zwischen 1-9 sein");
		ROS_INFO("Bitte erneut eingeben");
		c = getch();
		c = c-48;
		}

       		// Daten werden als arry versindet ( [0] = Spielfeldnummer, [1] = Spielernummer
		next_move.data = {c, player};

      		pub_next_move.publish(next_move);
		msg_valid = -1;

		//Wechsel von Spieler 1 auf 2 oder umgekehrt
		if (player == 1)
		{
        	player = player +1;
       		}
	     	else
	     	{
        	player = player -1;
       		}

    	//ROS_INFO("Das Spielfeld ist %d", next_move.data[0]);
    	}

     ros::spinOnce();
     loop_rate.sleep();
     ++count;
  }

  return 0;
}
