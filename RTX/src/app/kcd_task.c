/* The KCD Task Template File */
#ifndef KCD_TASK
#define KCD_TASK
#include "common.h"
#include "Serial.h"
#include "k_inc.h"
#include "rtx.h"
#include "printf.h"

U32 *create_command_map() {
	U32 *temp = mem_alloc((26+26+10) * sizeof(U32));
	for(int i = 0; i < 26+26+10; i++) temp[i] = 0;
	return temp;
}

char get_mapping(char asc) {
	if(asc < 58) return (asc-48);
	else if(asc < 91) return (asc-65) + 10;
	else return asc-97+35;
}

void print_mapping(void *buf) {
	U32 *buf1 = (U32 *) buf;
	printf("MAP PRINT BEGIN\r\n");
	for(int i = 0; i < 26+26+10; i++) printf("%d ", buf1[i]);
	printf("\r\nMAP PRINT FINISH\r\n");
}

void print_buffer(char *command_buf, U32 length){
	for (int i = 0; i < length; i++){
		printf("%c", command_buf[i]);
	}
	printf("\r\n");
}

void kcd_task(void){
	mbx_create(KCD_MBX_SIZE);									// Create mailbox, command map
	U32 *command_map = create_command_map();

	char *command_buffer = mem_alloc(sizeof(RTX_MSG_HDR) + 64); // SIZE 72 - init command buffer
	command_buffer += sizeof(RTX_MSG_HDR); 						// Increment address by 8 to avoid header
	for(int i = 0; i < 64; i++) command_buffer[i] = 0;

	void *buf = mem_alloc(sizeof(RTX_MSG_HDR) + 1); 			// SIZE 9

	static char cannot_process[30] = "Command cannot be processed\r\n";
	static char command_invalid[18] = "Invalid Command\r\n";

	char first_char = 0; U32 command_length = 0; U32 buffer_index = 0; U8 new_command = 1;
	U32 map_index = 0; U32 msg_type = 0; char command = 0;

	while(1){
		task_t sender;
		if(recv_msg(&sender, buf, sizeof(RTX_MSG_HDR) + 1) == RTX_OK){
			command = *((char *)buf + sizeof(RTX_MSG_HDR)); 	// Retrieve first byte
			msg_type = cq_get_type((RTX_MSG_HDR *) buf);

			if (msg_type == KCD_REG){							// Register command
				map_index = get_mapping(command);
				if(map_index > 51) continue;
				command_map[map_index] = sender;
//				print_mapping(command_map);
			}
			else if(msg_type == KEY_IN){						// Forward Putty input keys to KCD task [echo]
				if(sender != TID_UART_IRQ) continue;

				if(command == 13){ 								// Enter key pressed
//					print_mapping(command_map);

					if(command_length > 64 || command_length == 0 || first_char != 37){		// Send command invalid to user
						SER_PutStr(1, command_invalid);
						command_length = 0; buffer_index = 0; new_command = 1;
						continue;
					}

					map_index = get_mapping(command_buffer[0]);

					if(map_index > 51 || command_map[map_index] == 0 || g_tcbs[command_map[map_index]].state == DORMANT){	// Error cases
						SER_PutStr(1, cannot_process);
						command_length = 0; buffer_index = 0; new_command = 1;
						continue;
					}

					for(int i = 0; i < buffer_index; i++){				// Error cases
						map_index = get_mapping(command_buffer[i]);
						if(map_index > 51){
							SER_PutStr(1, command_invalid);
							command_length = 0; buffer_index = 0; new_command = 1;
							continue;
						}
					}

					map_index = get_mapping(command_buffer[0]);			// Grab mapped index of first char
					task_t reciever_tid = command_map[map_index];

					command_buffer -= sizeof(RTX_MSG_HDR);				// De-increment ptr to get to header

					RTX_MSG_HDR *temp = (RTX_MSG_HDR *)command_buffer;
					temp->length = sizeof(RTX_MSG_HDR) + buffer_index;
					temp->type = KCD_CMD;
					if(send_msg(reciever_tid, (const void *)command_buffer)==RTX_ERR){
						SER_PutStr(1, cannot_process);
						command_buffer += sizeof(RTX_MSG_HDR);				// Re-increment ptr to get to data

						command_length = 0; buffer_index = 0; new_command = 1;

						continue;

					}

					command_buffer += sizeof(RTX_MSG_HDR);				// Re-increment ptr to get to data

					command_length = 0; buffer_index = 0; new_command = 1;
				}
				else if(new_command){									// Current char is first in message - start incrementing after recording it
					first_char = command;
					new_command = 0;
					command_length++;
				}
				else if(command_length >= 64){
					command_length++;
					continue;
				}// If buffer full, continue but don't error until enter pressed
				else{
					command_buffer[buffer_index] = command;
//					print_mapping(command_map);
					buffer_index++;
					command_length++;
//					print_buffer(command_buffer, command_length);
				}
			}

		}
//		else{
//			printf("Doesn't enter\r\n");
//		}
	}
}

#endif
