#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

int open_serial_port(const char * device ,uint32_t baud_rate)
{
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror(device);
        return -1;
    }
    
    int result = tcflush(fd, TCIOFLUSH);
    if (result)
    {
        perror("tcflush failed");
    } 

    struct termios options;
    result = tcgetattr(fd, &options);
    if(result)
    {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }
    
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO  | ECHONL| ICANON| ISIG | IEXTEN);

    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN]  = 0;

    switch (baud_rate)
    {
        case 4800:  cfsetospeed(&options, B4800);   break;
        case 9600:  cfsetospeed(&options, B9600);   break;
        case 19200: cfsetospeed(&options, B19200);  break;
        case 38400: cfsetospeed(&options, B38400);  break;
        case 115200:cfsetospeed(&options, B115200); break;
        default:
            fprintf(stderr, "warning: baud rate %u is not supported, using 38400.\n", baud_rate);
            cfsetospeed(&options, B38400);
            break; 
    }

    cfsetispeed(&options, cfgetospeed(&options));

    result = tcsetattr(fd, TCSANOW, &options);
    if(result)
    {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }
    return fd;
}

int write_port(int fd, uint8_t *buffer, size_t size)
{
    ssize_t result = write(fd, buffer, size);
    if (result != (ssize_t)size)
    {
        perror("failed to write to port");
        return -1;
    }
    return 0;
}

ssize_t read_port(int fd, uint8_t *buffer, size_t size)
{
    size_t received = 0;
    while(received < size)
    {
        ssize_t r = read(fd, buffer + received, size - received);
        if(r<0)
        {
            perror("failed to read from port");
            return -1;
        }
        if(r==0){
            break;
        }
        received += r;
    }
    return received;


}
int jrk_set_target(int fd, uint16_t target)
{
  if (target > 4095) { target = 4095; }
  uint8_t command[2];
  command[0] = 0xC0 + (target & 0x1F);
  command[1] = (target >> 5) & 0x7F;
  return write_port(fd, command, sizeof(command));
}

int jrk_get_variable(int fd, uint8_t offset, uint8_t * buffer, uint8_t length)
{
  uint8_t command[] = { 0xE5, offset, length };
  int result = write_port(fd, command, sizeof(command));
  if (result) { return -1; }
  ssize_t received = read_port(fd, buffer, length);
  if (received < 0) { return -1; }
  if (received != length)
  {
    fprintf(stderr, "read timeout: expected %u bytes, got %zu\n",
      length, received);
    return -1;
  }
  return 0;
}

int jrk_get_target(int fd)
{
  uint8_t buffer[2];
  int result = jrk_get_variable(fd, 0x02, buffer, sizeof(buffer));
  if (result) { return -1; }
  return buffer[0] + 256 * buffer[1];
}

int jrk_get_feedback(int fd)
{
  uint8_t buffer[2];
  int result = jrk_get_variable(fd, 0x04, buffer, sizeof(buffer));
  if (result) { return -1; }
  return buffer[0] + 256 * buffer[1];
}

int main()
{
    const char * device = "/dev/cu.usbserial-1420";
  
  uint32_t baud_rate = 9600;
 
  int fd = open_serial_port(device, baud_rate);
  if (fd < 0) { return 1; }
 
  int feedback = jrk_get_feedback(fd);
  if (feedback < 0) { return 1; }
 
  printf("Feedback is %d.\n", feedback);
 
  int target = jrk_get_target(fd);
  if (target < 0) { return 1; }
  printf("Target is %d.\n", target);
 
  int new_target = (target < 2048) ? 2248 : 1848;
  printf("Setting target to %d.\n", new_target);
  int result = jrk_set_target(fd, new_target);
  if (result) { return 1; }
 
  close(fd);
  return 0;
}
