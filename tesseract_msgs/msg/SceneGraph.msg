string id # Scene Graph Name
string root # Root Link Name

tesseract_msgs/Link[] links
tesseract_msgs/Joint[] joints

string[] invisible_links # List of links which are hidden
string[] disabled_collision_links # List of links which have collision disabled

tesseract_msgs/AllowedCollisionEntry[] acm # The allowed collision matrix
