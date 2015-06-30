#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <learning_actionlib/FibonacciAction.h>


//class containing the client
class ActionTestClient{
public:
  ActionTestClient(std::string name):
  //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
          ac("fibonacci", true),
          //Stores the name
          action_name(name){
    //Get connection to a server
    ROS_INFO("%s Waiting For Server...", action_name.c_str());
    //Wait for the connection to be valid
    ac.waitForServer();
    ROS_INFO("%s Got a Server...", action_name.c_str());
  }
  // Called once when the goal completes

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const learning_actionlib::FibonacciResultConstPtr & result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Result: %d", result->statusCode);
    ros::shutdown();
  };

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active...");
  };

  // Called every time feedback is received for the goal
  void feedbackCb(const learning_actionlib::FibonacciFeedbackConstPtr & feedback)
  {
    ROS_INFO("Got Feedback of Progress to Goal: %s", feedback->feedbackString.c_str());
  };

  //Send a goal to the server
  void send(learning_actionlib::beaconSetting &b){
    //action_test::TestGoal newGoal;
    learning_actionlib::FibonacciGoal newGoal;

   // learning_actionlib::beaconSetting b;

    newGoal.bSet.beaconValues.push_back(b);

    //newGoal.request = goal;
    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(newGoal, boost::bind(&ActionTestClient::doneCb, this, _1, _2),
                boost::bind(&ActionTestClient::activeCb, this),
                boost::bind(&ActionTestClient::feedbackCb, this, _1));
  }

private:

actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> ac;
  //actionlib::SimpleActionClient<action_test::TestAction> ac;
  std::string action_name;
};

//Used by ROS to actually create the node. Could theoretically spawn more than one client
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_action_client");

  //Initialize the client
  ActionTestClient client(ros::this_node::getName());

  learning_actionlib::beaconSetting sampleGoal;

  sampleGoal.beaconID=11;
  sampleGoal.X=44;
  sampleGoal.Y=55;
  sampleGoal.Z=66;

  //Send the goal to the server
  client.send(sampleGoal);
  ROS_INFO("Sent Goal %d To Server...", sampleGoal);

  ros::spin();
  return 0;
}


int main2 (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  learning_actionlib::FibonacciGoal goal;

  //goal.order = 20;
  learning_actionlib::beaconSetting b;

  goal.bSet.beaconValues.push_back(b);

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());

  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
