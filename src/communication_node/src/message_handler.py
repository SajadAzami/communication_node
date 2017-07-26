"""Message Handler.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

Plays the server role in communication_node

Relations
----------
subscribes from /message_server topic,
publishes on corresponding nodes /ns/message_status topic

"""
# TODO design subscriber and publisher
rospy.loginfo(trans)
