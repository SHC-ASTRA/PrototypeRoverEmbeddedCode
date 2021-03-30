#include <communication_standard.h>

/*
Header
    Type Flag               1 Byte
    Length                  2 Bytes
    XOR Parity Check        1 Byte

Body
    Data

*/

void construct_header(char* message, int body_length, char type_flag)
{
    message[0] = type_flag;
    message[1] = (char) ((body_length >> 8) & 255);
    message[2] = (char) (body_length & 255);

    char parity = 0;
    for (int i = 0; i < body_length; i++)
    {
        parity ^= message[i+4];
    }

    message[3] = parity;
}

Message construct_status_message(char* status, int status_length)
{
    char* data = new char[status_length + 4];
    for (int i = 0; i < status_length; i++)
    {
        data[i+4] = status[i];
    }
    construct_header(data, status_length, STATUS);

    return (Message) { data, 4 + status_length};
}