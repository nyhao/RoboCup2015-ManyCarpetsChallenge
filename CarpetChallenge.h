/* CarpetChallenge.h
 * @author Yi Hao Ng
 * @author Philippe Schwendener
 */

bool ballWasSeen = false;
bool start;
bool turnLeft;

float postAngle1;
float postAngle2;
float goalAngle;
float alpha;
float previousRobotRotation;

Vector2<> goalPost1;
Vector2<> goalPost2;
Vector2<> distPosts;
Vector2<> alignTarget;
Vector2<> ballToPost;
Vector2<> goalMiddle;
Vector2<> targetImmediately;
Vector2<> targetKick;
Vector2<> trueLeftPost = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal);
Vector2<> trueRightPost = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal);

Vector2<> robotPosOri;
float robotRotOri;
 
 option(CarpetChallenge)
{
	//Initial State
	initial_state(start)
  {
    transition
    {
			if(state_time > 100)
			{
       start = true;
				goto searchBall;
			}
    }
		action
		{	
		}
  }
	
	//First state to go in.
	//If the ball was seen go towards Ball (in state walkToBall)
	state(searchBall)
  {
    transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() < 300)
			{
				goto turnToBall;
				
			}
				
		}
    action
    {
			SearchForBall();
    }
  }
	
  //Turn in the direction of the Ball!
	state(turnToBall)
  {
    transition
		{
			if(fabs(theBallModel.estimate.position.angle()) < 0.1)
				goto walkToBall;
		}
    action
    {
        lookTargetRelative = theBallModel.estimate.position;
        theHeadControlMode = HeadControl::lookAtTarget;
        theMotionRequest.motion = MotionRequest::walk;
        theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
        theMotionRequest.walkRequest.target.translation = Vector2<>(0,0);
        theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.position.angle();
        theMotionRequest.walkRequest.speed = Pose2D(1, 100, 100);
    }
  }
	
	state(walkToBall)
  {
    transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 5000)
			{
				goto searchBall;
			}
			else if(theBallModel.estimate.position.abs() < 500)
			{
        OUTPUT(idText, text, "Search Goal!");
          goto searchGoal;
			}
			if(option_time > 45000)
			{
				targetImmediately = relToGlo(Vector2<>(1000,0));
				goto kickImmediately;
			}
				
		}
    action
    {
			lookTargetRelative = theBallModel.estimate.position;
			theHeadControlMode = HeadControl::lookAtTarget;
			theMotionRequest.motion = MotionRequest::walk;
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			theMotionRequest.walkRequest.target.translation = theBallModel.estimate.position;
			theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.position.angle();
			theMotionRequest.walkRequest.speed = Pose2D(1, 100, 100);
			
    }
  }
  
  //Now we are at the Ball. Search the goal.
	state(searchGoal)
	{
		transition
		{
      //If we looked in all directions but goal was not seen. (e.g. if goal is behind robot), then walk around the ball
      if(state_time > 5000)
      {
        findGoalDone = false;
        OUTPUT(idText, text, "WalkAroundBall and Search");
        goto walkAroundBallSearch;
      }

			if(theGoalPercept.timeWhenGoalPostLastSeen > 0)
			{
      //Computing some variables that are used to turn towards goal.
				goalPost1 = theGoalPercept.goalPosts[0].positionOnField;
				//goalPost2 = theGoalPercept.goalPosts[1].positionOnField;
       //goalMiddle = (goalPost1 + goalPost2)/2;
			 goalMiddle = goalPost1;
 
       postAngle1 = goalPost1.angle();
				//postAngle2 = goalPost2.angle();
				//goalAngle = (postAngle1 + postAngle2)/2;
				goalAngle = postAngle1;
        
				//distPosts = goalPost1 - goalPost2;
				
       // Compute the global angle to the middle of the goal
				alpha = theRobotPose.rotation + goalAngle;
				if (alpha > pi)
				{
					float temp = alpha - pi;
					alpha = -pi + temp;
				}
				else if (alpha < -pi)
				{
					float temp = fabs(alpha + pi);
					alpha = pi - temp;
				}
        
				//check how trustfull the information about the posts is...
				//they have a max. dist of 2m, if dist between posts is bigger info is not trustfull.
				//if (distPosts.abs() < 2000)
				//{
					if (goalAngle <= 0)
					{
           previousRobotRotation = theRobotPose.rotation;
						OUTPUT(idText, text, "Left");
						goto walkAroundBallLeft;
					}
					else if(goalAngle > 0)
					{
           previousRobotRotation = theRobotPose.rotation;
						OUTPUT(idText, text, "Right");
						goto walkAroundBallRight;
					//}

				}
				else
				{
					goto searchBall;
				}
			}
			if(option_time > 45000)
			{
				targetImmediately = relToGlo(Vector2<>(1000,0));
				goto kickImmediately;
			}
		}
		action
		{
			Stand();
			theHeadControlMode = HeadControl::findGoalCarpetChallenge;
		}
	}
	
	state(walkAroundBallSearch)
	{
	transition
	{
		
		if(state_time > 4000)
			goto turnToBall;
      
    //if(fabs(theRobotPose.rotation - previousRobotRotation) > 0.1 && fabs(theRobotPose.rotation - pi) > 0.1 )
      //goto searchGoal;
	}
	action
	{
		lookTargetRelative = theBallModel.estimate.position;
		theHeadControlMode = HeadControl::lookAtTarget;
    
		theMotionRequest.motion = MotionRequest::walk;
		theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target.translation = Vector2<> (0 ,100);
		theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.position.angle();
		theMotionRequest.walkRequest.speed = Pose2D(1, 100, 100);
	}
	}
	state(walkAroundBallLeft)
	{
	transition
	{
    //Because if he walks to long he falls down some times
    if(state_time > 5000)
      goto lookToGoal;
		if(fabs(theRobotPose.rotation - alpha) < 0.05)
			goto lookToGoal;
      
    //If self-localization jumps around, start from begining  
    //if(fabs(theRobotPose.rotation - previousRobotRotation) > 0.1 && fabs(theRobotPose.rotation - pi) > 0.1 )
      //goto searchGoal;
    
    previousRobotRotation = theRobotPose.rotation;
		
		if(option_time > 45000)
		{
			targetImmediately = relToGlo(Vector2<>(1000,0));
			goto kickImmediately;
		}
	}
	action
	{
		lookTargetRelative = theBallModel.estimate.position;
		theHeadControlMode = HeadControl::lookAtTarget;
		
		theMotionRequest.motion = MotionRequest::walk;
		theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
		theMotionRequest.walkRequest.target.translation = Vector2<> (0 ,+100);
		theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.position.angle();
		theMotionRequest.walkRequest.speed = Pose2D(1, 100, 100);
	}
	}
	state(walkAroundBallRight)
	{
	transition
	{
    if(state_time > 5000)
      goto lookToGoal;
    if(fabs(theRobotPose.rotation - alpha) < 0.05)
			goto lookToGoal;
      
    //if(fabs(theRobotPose.rotation - previousRobotRotation) > 0.1 && fabs(theRobotPose.rotation - pi) > 0.1 )
      //goto searchGoal;
      
    previousRobotRotation = theRobotPose.rotation;
		
		if(option_time > 45000)
		{
			targetImmediately = relToGlo(Vector2<>(1000,0));
			goto kickImmediately;
		}
	}
	action
	{
		lookTargetRelative = theBallModel.estimate.position;
		theHeadControlMode = HeadControl::lookAtTarget;
		
		theMotionRequest.motion = MotionRequest::walk;
		theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
		theMotionRequest.walkRequest.target.translation = Vector2<> (0 ,-100);
		theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.position.angle();
		theMotionRequest.walkRequest.speed = Pose2D(1, 100, 100);
	}
	}
  
  //This state is to check if the goal is really infront of the robot.
  state(lookToGoal)
  {
  transition
	{
      //OUTPUT(idText, text, theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen));
      if(state_time > 5000)
      {
        findGoalDone = false;
        OUTPUT(idText, text, "WalkAroundBall and Search");
        goto walkAroundBallSearch;
      }  
			if(theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 500)
			{
				OUTPUT(idText, text, "LookToGoal: GoalPostWasSeen!");
				goalPost1 = theGoalPercept.goalPosts[0].positionOnField;
				goalPost2 = theGoalPercept.goalPosts[1].positionOnField;
				
				/*OUTPUT(idText, text, relToGlo(goalPost1).x);
				OUTPUT(idText, text, relToGlo(goalPost1).y);
				OUTPUT(idText, text, relToGlo(goalPost2).x);
				OUTPUT(idText, text, relToGlo(goalPost2).y);*/
				
				/*if((goalPost1 -  trueLeftPost).abs() < 0.1)	
					goalPost1 = gloToRel(trueLeftPost);
				else if((goalPost1 -  trueRightPost).abs() < 0.1)	
					goalPost1 = gloToRel(trueRightPost);
					
				if((goalPost2 -  trueLeftPost).abs() < 0.1)	
					goalPost2 = gloToRel(trueLeftPost);
				else if((goalPost2 -  trueRightPost).abs() < 0.1)	
					goalPost2 = gloToRel(trueRightPost);*/
					
				int postCount;
				postCount = theGoalPercept.goalPosts.size();
				OUTPUT(idText, text, postCount);
				
       goalMiddle = (goalPost1 + goalPost2)/2;
 
       postAngle1 = goalPost1.angle();
				postAngle2 = goalPost2.angle();
				goalAngle = (postAngle1 + postAngle2)/2;
        
				distPosts = goalPost1 - goalPost2;
				
       
				//check how trustfull the information about the posts is...
				//they have a max. dist of 2m, if dist between posts is bigger info is not trustfull.
				if (distPosts.abs() < 2000)
				{
          if (fabs(goalAngle) <= 0.4)
          {
							OUTPUT(idText, text, "Post 1:");
							OUTPUT(idText, text, relToGlo(goalPost1).x);
							OUTPUT(idText, text, relToGlo(goalPost1).y);
							OUTPUT(idText, text, "Post 2:");
							OUTPUT(idText, text, relToGlo(goalPost2).x);
							OUTPUT(idText, text, relToGlo(goalPost2).y);

							
							targetKick = relToGlo((goalPost1 + goalPost2)/2);
							OUTPUT(idText, text, "Kick Target x:");
							OUTPUT(idText, text, targetKick.x);
							OUTPUT(idText, text, "Kick Target y:");
							OUTPUT(idText, text, targetKick.y);							
            goto kick;
          }
          else
          {
            goto searchGoal;
          }
				}
				else
				{
          OUTPUT(idText, text, "Posts to far away");
					goto searchBall;
				}
			}
			if(option_time > 45000)
			{
				targetImmediately = relToGlo(Vector2<>(1000,0));
				goto kickImmediately;
			}
		}
		action
		{
			Stand();
			theHeadControlMode = HeadControl::findGoalCarpetChallenge;
		}
  }
	
	state(kick)
  {
    transition
		{
			if (action_done)
        goto searchBall;
		}
    action
    {

			
			WalkAndKickLeftFoot(targetKick);
			
    }
  }
	state(kickImmediately)
  {
    transition
		{
			if (action_done)
        goto searchBall;
		}
    action
    {
			WalkAndKickLeftFoot(targetImmediately);
			
    }
  }
}
