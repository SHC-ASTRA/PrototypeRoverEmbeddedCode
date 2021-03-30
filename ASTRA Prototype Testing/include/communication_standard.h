// Message Type Flag Declarations
// These should be declared in numerical order
const char STATUS = 0;
const char FLAG_CHECK = 1;

// Message Struct
// Allows the message constructors to pass data back to the caller
struct Message
{
    char* data;
    int length;
};

// Function Declarations
Message construct_status_message(char* status, int status_length);