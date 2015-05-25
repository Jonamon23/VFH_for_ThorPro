function getGoal(goal_msg) % Goal Callback

global goal;
goal.x =goal_msg.getX;
goal.y =goal_msg.getY;