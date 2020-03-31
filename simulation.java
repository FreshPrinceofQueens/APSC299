import java.util.Scanner;
public class simulation {
	
	Scanner sc = new Scanner(System.in);
	
	private static final int LEFT = 0;
	private static final int RIGHT = 1;
	private static final int FORWS = 2;
	private static final int BACKWS = 3;
	private static final int NORTH = 0;
	private static final int SOUTH = 180;
	private static final int EAST = 90;
	private static final int WEST = 270;
	
	private static final float wheelSpeed = 0.15f;
	private static final float wallLength = 1.7f;
	
	private static final int LOW = 0;
	private static final int HIGH = 1;
	
	private static final String IDLE = "IDLE";
	private static final String SEARCHING = "SEARCHING";
	private static final String FOUND = "FOUND";
	private static final String DRIVING = "DRIVING";
	
	private float cordX  =(float) 0.0, cordY = 0.0f;
	private String STATE ="UNKNOWN";
	private String STATE_OTHERS ="UNKNOWN";
	
	private boolean taskComplete = false, awaitingInstructions = false;
	private int invertCSG = 0;
	private int orientation = NORTH;
	private int pre_orientation = -1;
	private int IRSENSOR = 0;
	private boolean toReact = false;
	private int stopSignal = 0;
	private int colorSignal = 0;
	private boolean toStop = false;
	
	private void proceed() {

		System.out.print("Press 1 to Proceed: ");
		Integer value = -1;
		while(value.intValue() != 1) {
			value = Integer.parseInt(sc.next());
		}
		
	}
	
	private void status(String s) {
		System.out.println("----------------------------------------------");
		System.out.println(s);
		proceed();
		System.out.println("----------------------------------------------");
	}
	
	
	private void stopWheels() {
		System.out.println("----------------------------------------------");
		System.out.println("Wheels are Stopped");
		System.out.println("----------------------------------------------");
	}
	
	private void forwards() {
		System.out.println("----------------------------------------------");
		System.out.println("Wheels are moving forward");
		System.out.println("----------------------------------------------");
	}
	
	private String oriToString(int input) {
		  String tobereturned = "POOJA WHAT IS THIS BEHAVIOUR???";
		  switch (input) {
		    case NORTH:
		      tobereturned = "NORTH";
		      break;

		    case SOUTH:
		      tobereturned = "SOUTH";
		      break;

		    case EAST:
		      tobereturned = "EAST";
		      break;

		    case WEST:
		      tobereturned = "WEST";
		      break;

		    case -1:
		      tobereturned = "UNINITIALIZED";
		      break;

		    default:
		      tobereturned = "UNKNOWNN";
		      break;
		  }
		  return tobereturned;
		}
	
	void orientationInfo() {
		System.out.println("----------------------------------------------");
		System.out.println("Previous Orientation is "+oriToString(pre_orientation));
		System.out.println("Orientation is "+oriToString(orientation));
		System.out.println("----------------------------------------------");
		}
	
	private void turnWheels(float angle, int dir) {
		System.out.println("----------------------------------------------");
		String direct = "";
		if(dir == RIGHT) direct = "RIGHT";
		else direct = "LEFT";
		
		stopWheels();
		int tempangle = (int)angle;
		orientationInfo();
		//proceed();
	      pre_orientation = orientation;
	      
	    System.out.println("Turning wheels: "+angle+","+direct);
	    //proceed();
	    
	      if (dir == RIGHT)  orientation = orientation + tempangle;
	      if (dir == LEFT) orientation = orientation + (360 - tempangle);
	      if (orientation >= 360) orientation = orientation - 360;
	      
	     orientationInfo();
	     //proceed();
	     stopWheels();
	     System.out.println("----------------------------------------------");
	      
	}
	
	boolean checkColorSignal() {
		System.out.println("----------------------------------------------");
		  boolean result = false;
		  if (colorSignal >= 1000)result = true; //colorsignal greater than 1000 when on base
		  else result = false;

		  if (invertCSG == 1) {
		    if (result == true) result = false;
		    if (result == false) result = true;
		  }
		  System.out.println("----------------------------------------------");
		  return result;
		  
		}
	
	void checkStopSignal() {
		System.out.println("----------------------------------------------");
		System.out.println("In checkStopSignal:");
		getSTATE();
		if(STATE.equals(DRIVING) || STATE.equals(FOUND)|| STATE.equals("UNKNOWN") ||toStop) stopSignal = 1;
		else stopSignal = 0;
		System.out.println("Stop signal is "+stopSignal);
		System.out.println("----------------------------------------------");
	}
	
	void printCoordinates() {
		System.out.println("(x: "+cordX+",y: "+cordY+")");
	}
	
	private int go(float distance, int dir) {
		System.out.println("----------------------------------------------");
		boolean exit0 = false,exit1 =false, exit2 = false, exit4 = false,exit5 = false;
		  
		  switch(dir) {
		  case LEFT:
			  System.out.println("going left");
		      turnWheels(90,LEFT);
		  break;
		  case RIGHT:
			  System.out.println("going RIGHT");
		      turnWheels(90,RIGHT);
		  break;
		  case BACKWS:
			  System.out.println("going BACL");
		      turnWheels(180,LEFT);
		  break;
		  case FORWS:
			  System.out.println("going forwards");
			  pre_orientation = orientation;
		  break;
		  }
		  
		  int exitstatus = -1;
		  //exit status
		  float addtoX = (float) 0.0, addtoY = (float) 0.0;
		  float time1 = (float) 0.0, time2 = (float) 0.0, diff = (float) 0.0, curdist = (float) 0.0; //initial variables

		  float timing = (float)(distance / wheelSpeed) * 1000;
		  
		  forwards();
		  while(true) {
			  System.out.println("In move while loop");
			  System.out.println("Force what exit status: ");
			  Integer value = Integer.parseInt(sc.next());
			  switch (value.intValue()) {
			    //exitstatus info
			    case 0:
			      exit0 = true;
			      break;
			    case 1:
			    	exit1 = true;
			      break;
			    case 2:
			    	exit2 = true;
			      break;
			    case 4:
			    	exit4 = true;
			      break;
			    case 5:
			    	exit5 = true;
			      break;
			  }
			  
			  //proceed();
			  time2 = (float)(time2 + 0.01);
			  diff = time2 - time1;
			  curdist = diff * wheelSpeed / 1000;
			  
			  //checkStopSignal();
			    if (exit5) {
			      //have been told to stop, need to break out of both while loops
			      exitstatus = 5;
			      break;
			    }
			    if (exit1) {
			      //ir sensor senses something close
			      exitstatus = 1;
			      break;
			    }
			    // System.out.print(toReact());
			    if (exit2) {

			      //bumpers are hit
			      exitstatus = 2;
			      break;
			    }

			     //read from ir sensor
			    if (exit4) {
			      exitstatus = 4;
			      break;
			    }

			    if (exit0) {
			      //drove full length
			      exitstatus = 0;
			      break;
			    }
		  }
		  stopWheels();
		  curdist = distance;
		  if (pre_orientation == -1) pre_orientation = NORTH;
		  switch (pre_orientation) {
		    case NORTH:
		      if (dir == FORWS) addtoY = curdist;
		      else if (dir == BACKWS) addtoY = curdist * -1;
		      else if (dir == RIGHT) addtoX = curdist;
		      else if (dir == LEFT) addtoX = curdist * -1;
		      break;

		    case EAST:
		      if (dir == FORWS) addtoX = curdist;
		      else if (dir == BACKWS) addtoX = curdist * -1;
		      else if (dir == RIGHT) addtoY = curdist * -1;
		      else if (dir == LEFT) addtoY = curdist;
		      break;

		    case SOUTH:
		      if (dir == FORWS) addtoY = curdist * -1;
		      else if (dir == BACKWS) addtoY = curdist;
		      else if (dir == RIGHT) addtoX = curdist * -1;
		      else if (dir == LEFT) addtoX = curdist;
		      break;

		    case WEST:
		      if (dir == FORWS) addtoX = curdist * -1;
		      else if (dir == BACKWS) addtoX = curdist;
		      else if (dir == RIGHT) addtoY = curdist;
		      else if (dir == LEFT) addtoY = curdist * -1;
		      break;
		  }

		  cordX = cordX + addtoX;
		  cordY = cordY + addtoY;
		  
		  switch (exitstatus) {
		    //exitstatus info
		    case 0:
		      System.out.println("Exit Status 0: Clear Run.");
		      break;
		    case 1:
		      System.out.println("Exit Status 1: IR Error.");
		      break;
		    case 2:
		      System.out.println("Exit Status 2: Bumper Barrier Error.");
		      break;
		    case 4:
		      System.out.println("Exit Status 4: Color signal asserted.");
		      break;
		    case 5:
		      System.out.println("Exit Status 5: Stop signal asserted.");
		      break;
		  }

		  printCoordinates();
		  System.out.println("----------------------------------------------");
		  return exitstatus;
	}

	private void forceSTATE(String stateIn) {
		STATE = stateIn;
		System.out.println("State is now: "+STATE);
	}
	
	private void forceSTATEOthers (String stateIn) {
		STATE_OTHERS = stateIn;
		System.out.println("STATE_OTHERS is now:"+STATE_OTHERS);
	}
	
	private void getSTATEOthers()
	{
		System.out.println("STATE_OTHERS is:"+STATE_OTHERS);
		System.out.println("change state? Yes-1 No-0");
		if(sc.next() == "1") forceSTATEOthers(sc.next());
	}
	
	private void getSTATE() {
		System.out.println("State is: "+STATE);
		System.out.println("change state? no (0) or type state you want");
		String value =sc.next();
		if(!value.equals("0")) forceSTATE(value);
	}
	
	private void driveOntoBase() {
//		System.out.println("In Drive onto Base");
//		proceed();
//		System.out.println("Drove onto base");
		
		int exitstatus = -1;
		  int turnangle = 90;
		  invertCSG = 1;
		  
		  while(exitstatus != 0) {
			  exitstatus = go(0.25f, FORWS);
			  
			  if(exitstatus == 4) {
				  while (colorSignal < 1000) {
					  System.out.print("Searching for base, assert color signal value to:");
					  Integer value = Integer.parseInt(sc.next());
					  colorSignal = value;
				        turnWheels(turnangle, RIGHT);
				        turnangle = turnangle + 90;

				        //continues turning and reading the colorsignal, thatis
				        //turns 90, reads from colorsignal, if it is not a greater than a 1000 then turn again
				      }
			  }
		  }
	}
	
	private void sendLocation() {
		System.out.println("Location has been sent to other robots");
	}

	void setOrientation(int desiredOrientation) {
		  int diff = desiredOrientation - orientation;
		  if (diff < 0) diff = 360 + diff;
		  turnWheels(diff, RIGHT);
		  pre_orientation = orientation;
		}
	
	void driveToLocation(float rec_x, float rec_y) {
		  float distx = rec_x - cordX;
		  float disty = rec_y - cordY;

		  //passed in coordinates are based on North orientation, therefore robot must be set
		  //to north orientation before being told to drive.
		  setOrientation(NORTH);

		  if (distx < 0) {
		    //needs to go in -x direction, therefore to the left
		    distx = distx * -1;
		    go(distx, LEFT);
		  }
		  else
		  {
		    //positive x direction therefore go right
		    go(distx, RIGHT);
		  }

		  setOrientation(NORTH);
		  if (disty < 0) {
		    //needs to go -y direction, therefore backwards
		    disty = disty * -1;
		    go(disty, BACKWS);
		  }
		  else
		  {
		    //positive y direction, therefore go forwards
		    go(disty, FORWS);
		  }
		  
		  printCoordinates();
		}

	void search() {
		
		int state = -1;
		  if (!taskComplete && !awaitingInstructions) {
		    //variable for exit status

		    int ending = -1;
		    //used to determine what side of the enclosure we are on, necessary for proper turns
		    //if at beginning, it is -1, when it reaches another end it is multiplied negative 1.
		    //beginning -> -1
		    //end -> 1;

		    float width = 0.0f;
		    //width of enclosure traversed.

		    float preCordX = -1, preCordY = -1;

		    invertCSG = 0;
		    //needs to stop when reached base, therefore when colorSignal is over 1000
		    //this means that we do not invertCSG
		    
		    while (true) {
		    	System.out.println("New Traversal******");
		        state = go(wallLength, FORWS);
		         //first begins with traveling the length of the enclosure

		         if (state == 0) {
		           //traversed the whole length
		           //at new point of enclosure therefore change ending.
		           ending = ending * -1;
		           status("ending is "+ending);
		           System.out.println("Max out width? Yes-1 No -0");
		           Integer value = Integer.parseInt(sc.next());
		           if(value.intValue() == 1) {
		        	   width = 0.60f;
		           }

		           if (width < 0.60) {
		             //individual sections are 60cm each therefore if wehave reached 60cm in width
		             //we can no longer traverse unless we will collide with another robot

		             if (ending == 1) {
		               //we are at end of enclosure
		               //offset to the right to search another part of our section
		               turnWheels(90, RIGHT);
		               go(0.15f, FORWS);
		               turnWheels(90, RIGHT);
		             }
		             else
		             {
		               //at the beginning therefore need to turn left to search more of the section
		               turnWheels(90, LEFT);
		               go(0.15f, FORWS);
		               turnWheels(90, LEFT);
		             }

		             width = (float)(width + 0.15f);
		             //increment the width
		           }
		           else
		           {
		             //individual sections are 60cm each therefore if wehave reached 60cm in width
		             //we can no longer traverse unless we will collide with another robot
		        	  status("can no longer traverse");
		             stopWheels();
		             forceSTATE(IDLE);
		             awaitingInstructions = true;
		             status("awaiting instructions is true");
		             break;
		           }
		         }
		         else   if (state == 2) {
		           //barrier error, recall fucntion
		           //call react bumpers to realign
		           
		         } 
		         else if (state == 4) {
		        	 status("state 4 within while loop, now breaking");
		           //have hit homebase need to run homebase code, break out of while loop
		           break;
		         }
		         else if (state == 5) {
		        	 status("state 5 within while loop, now breaking");
		           //have encountered stop signal therefore need to break out of while loop
		           break;
		         }

		       } //end while loop
		  }
		  
		  if(awaitingInstructions && !taskComplete){
		      //needs to be told what state to be put in next 
		      status("awaiting instructions and task is incomplete");
			  while(true){
		        checkStopSignal();
		        if(stopSignal == 1){
		          state = 5;
		          awaitingInstructions = false;
		          break;
		        }
		      }
		    }

		    if (state== 4 && !taskComplete) {
		      //need to go to base, run drive to base function
		      //sendStopSignal();
		    	status("state 4 and task is not complete");
		    	forceSTATEOthers(FOUND);
		    	forceSTATE(FOUND);
		      driveOntoBase(); //drive onto the base
		      sendLocation(); //send our location to other robots
		      forceSTATE(IDLE);//change our state to idle since now on board
		      taskComplete = true;
		      status("task has been set to complete");
		    }
		  
		    if (state == 5 && !taskComplete) {
		    	status("stop signal asserted and task is incomplete");
		        while (!STATE.equals(DRIVING)) { //waiting to be put into driving state if not already in driving state
		        	getSTATE();
		        	
		        }

		        //once in driving state, get coordinates and drive to location.
		        System.out.println("Target coordinates are sampled");
		        float rec_cordX = 2;
		        float rec_cordY = 3;
		        driveToLocation(rec_cordX, rec_cordY);

		        forceSTATE(IDLE); //let other memebers know we have reached the base
		        stopWheels(); //stop the wheels of the robot
		        taskComplete = true; //set taks complete to true so robot no longer conitnues to traverse
		        status("task is now complete");
		      }
		    
		    if(taskComplete && (awaitingInstructions == false)) {
		    	System.out.println("Task Complete");
		    	System.exit(0);
		    }
	}
	
	public static void main(String[] args) {
		simulation sm = new simulation();
		while(true) {
		sm.search();
		}
		
	}
}
		
		
