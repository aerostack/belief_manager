# Brief
This process is responsible for storing beliefs in the belief memory and executing queries against the current beliefs in memory.

# Services
- **add_belief** ([aerostack_msgs/AddBelief](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/AddBelief.srv))  
Adds a belief to the belief memory.

- **remove_belief** ([aerostack_msgs/RemoveBelief](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/RemoveBelief.srv))  
Removes a belief from the belief memory.

- **query_belief** ([aerostack_msgs/QueryBelief](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/QueryBelief.srv))  
Performs unification of a belief expression against the belief memory and returns a boolean with whether it was successful an a list of the variables it unified with their corresponding values.

- **check_belief_format** ([aerostack_msgs/CheckBeliefFormat](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/CheckBeliefFormat.srv))  
Returns a boolean indicating whether the given belief is correctly formed.

- **generate_id** ([droneMsgsROS/GenerateID](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/GenerateID.srv))  
Returns the identifier generated, a boolean which value means whether the identifier has been correctly generated and a string in case the boolean is false which contains the error.

- **query_last_generated_id** ([droneMsgsROS/QueryLastGeneratedID](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/QueryLastGeneratedID.srv))  
Returns the last identifier generated, a boolean which value means whether the identifier has  been correctly queried and a string in case the boolean is false which contains the error.



# Published topics
- **all_beliefs** ([aerostack_msgs/ListOfBeliefs](https://bitbucket.org/joselusl/dronemsgsros/src/master/msg/AllBeliefs.msg))  
All of the beliefs currently stored in the belief memory. It is published each time any belief is added or removed.

- **emergency_event** ([aerostack_msgs/StringStamped](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/5fcf0e3de4e41504bbf610fa5345ee108f6bf19f/msg/StringStamped.msg))  
Emergency event.

## Configuration file

The process uses a configuration file in YAML format called `belief_manager_config.yaml`.

This is an example of the content of the configuration file:

    predicate_semantics:
      - predicate_name:          item_grasped
        mutual_exclusive_values: no
        maximum_values:          10    

    emergency_events:
      - predicate_name:          battery_level
        emergency_value:         low
      - predicate_name:          collision_course

---
