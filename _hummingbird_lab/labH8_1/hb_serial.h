/**
 * @file hb_serial.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Defines the serial protocol for communicating with the hummingbird hardware
 */

#ifndef HB_SERIAL_H
#define HB_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//==============================================================================
// message types
//==============================================================================

typedef enum
{
  HB_SERIAL_MSG_ENCODER,
  HB_SERIAL_MSG_IMU,
  HB_SERIAL_MSG_SETPOINT,
  HB_SERIAL_MSG_ARM_STATUS,
  HB_SERIAL_MSG_ZERO,
  HB_SERIAL_NUM_MSGS
} hb_msg_type_t;

//==============================================================================
// message definitions
//==============================================================================

typedef struct
{
  float yaw;
  float pitch;
  float roll;
} hb_serial_encoder_msg_t;

typedef struct
{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} hb_serial_imu_msg_t;

typedef struct
{
  float left;
  float right;
} hb_serial_setpoint_msg_t;

typedef struct
{
  bool armed;
} hb_serial_arm_status_msg_t;

typedef struct
{
  bool success;
} hb_serial_zero_msg_t;

// payload lengths
static constexpr float HB_SERIAL_PAYLOAD_LEN[] = {
  3*sizeof(float),
  6*sizeof(float),
  2*sizeof(float),
  sizeof(bool),
  sizeof(bool)
};
static constexpr size_t HB_SERIAL_MAX_PAYLOAD_LEN = 6*sizeof(float);

//==============================================================================
// generic message type
//==============================================================================

static constexpr uint8_t HB_SERIAL_MAGIC = 0xA5;

typedef struct __attribute__((packed))
{
  uint8_t magic;
  uint8_t type;
  uint8_t payload[HB_SERIAL_MAX_PAYLOAD_LEN];
  uint8_t crc;
} hb_serial_message_t;

static constexpr size_t HB_SERIAL_MAX_MESSAGE_LEN = sizeof(hb_serial_message_t);

//==============================================================================
// utility functions
//==============================================================================

// source: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gab27eaaef6d7fd096bd7d57bf3f9ba083
inline uint8_t hb_serial_update_crc(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

inline void hb_serial_finalize_message(hb_serial_message_t *msg)
{
  msg->magic = HB_SERIAL_MAGIC;

  uint8_t crc = 0;
  crc = hb_serial_update_crc(crc, msg->magic);
  crc = hb_serial_update_crc(crc, msg->type);
  for (size_t i = 0; i < HB_SERIAL_PAYLOAD_LEN[msg->type]; i++)
  {
    crc = hb_serial_update_crc(crc, msg->payload[i]);
  }

  msg->crc = crc;
}

inline size_t hb_serial_send_to_buffer(uint8_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(dst + offset, &src->magic, sizeof(src->magic)); offset += sizeof(src->magic);
  memcpy(dst + offset, &src->type,  sizeof(src->type));  offset += sizeof(src->type);
  memcpy(dst + offset, src->payload, HB_SERIAL_PAYLOAD_LEN[src->type]); offset += HB_SERIAL_PAYLOAD_LEN[src->type];
  memcpy(dst + offset, &src->crc,   sizeof(src->crc)); offset += sizeof(src->crc);
  return offset;
}

//==============================================================================
// encoder message
//==============================================================================

inline void hb_serial_encoder_msg_pack(hb_serial_message_t *dst, const hb_serial_encoder_msg_t *src)
{
  dst->type = HB_SERIAL_MSG_ENCODER;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->yaw,   sizeof(src->yaw));   offset += sizeof(src->yaw);
  memcpy(dst->payload + offset, &src->pitch, sizeof(src->pitch)); offset += sizeof(src->pitch);
  memcpy(dst->payload + offset, &src->roll,  sizeof(src->roll));
  hb_serial_finalize_message(dst);
}

inline void hb_serial_encoder_msg_unpack(hb_serial_encoder_msg_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->yaw,   src->payload + offset, sizeof(dst->yaw));   offset += sizeof(dst->yaw);
  memcpy(&dst->pitch, src->payload + offset, sizeof(dst->pitch)); offset += sizeof(dst->pitch);
  memcpy(&dst->roll,  src->payload + offset, sizeof(dst->roll));
}

inline size_t hb_serial_encoder_msg_send_to_buffer(uint8_t *dst, const hb_serial_encoder_msg_t *src)
{
  hb_serial_message_t msg;
  hb_serial_encoder_msg_pack(&msg, src);
  return hb_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// IMU message
//==============================================================================

inline void hb_serial_imu_msg_pack(hb_serial_message_t *dst, const hb_serial_imu_msg_t *src)
{
  dst->type = HB_SERIAL_MSG_IMU;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->accel_x, sizeof(src->accel_x)); offset += sizeof(src->accel_x);
  memcpy(dst->payload + offset, &src->accel_y, sizeof(src->accel_y)); offset += sizeof(src->accel_y);
  memcpy(dst->payload + offset, &src->accel_z, sizeof(src->accel_z)); offset += sizeof(src->accel_z);
  memcpy(dst->payload + offset, &src->gyro_x,  sizeof(src->gyro_x));  offset += sizeof(src->gyro_x);
  memcpy(dst->payload + offset, &src->gyro_y,  sizeof(src->gyro_y));  offset += sizeof(src->gyro_y);
  memcpy(dst->payload + offset, &src->gyro_z,  sizeof(src->gyro_z));
  hb_serial_finalize_message(dst);
}

inline void hb_serial_imu_msg_unpack(hb_serial_imu_msg_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->accel_x, src->payload + offset, sizeof(dst->accel_x)); offset += sizeof(dst->accel_x);
  memcpy(&dst->accel_y, src->payload + offset, sizeof(dst->accel_y)); offset += sizeof(dst->accel_y);
  memcpy(&dst->accel_z, src->payload + offset, sizeof(dst->accel_z)); offset += sizeof(dst->accel_z);
  memcpy(&dst->gyro_x,  src->payload + offset, sizeof(dst->gyro_x));  offset += sizeof(dst->gyro_x);
  memcpy(&dst->gyro_y,  src->payload + offset, sizeof(dst->gyro_y));  offset += sizeof(dst->gyro_y);
  memcpy(&dst->gyro_z,  src->payload + offset, sizeof(dst->gyro_z));
}

inline size_t hb_serial_imu_msg_send_to_buffer(uint8_t *dst, const hb_serial_imu_msg_t *src)
{
  hb_serial_message_t msg;
  hb_serial_imu_msg_pack(&msg, src);
  return hb_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// setpoint message
//==============================================================================

inline void hb_serial_setpoint_msg_pack(hb_serial_message_t *dst, const hb_serial_setpoint_msg_t *src)
{
  dst->type = HB_SERIAL_MSG_SETPOINT;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->left,  sizeof(src->left)); offset += sizeof(src->left);
  memcpy(dst->payload + offset, &src->right, sizeof(src->right));
  hb_serial_finalize_message(dst);
}

inline void hb_serial_setpoint_msg_unpack(hb_serial_setpoint_msg_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->left,  src->payload + offset, sizeof(dst->left)); offset += sizeof(dst->left);
  memcpy(&dst->right, src->payload + offset, sizeof(dst->right));
}

inline size_t hb_serial_setpoint_msg_send_to_buffer(uint8_t *dst, const hb_serial_setpoint_msg_t *src)
{
  hb_serial_message_t msg;
  hb_serial_setpoint_msg_pack(&msg, src);
  return hb_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// arm status message
//==============================================================================

inline void hb_serial_arm_status_msg_pack(hb_serial_message_t *dst, const hb_serial_arm_status_msg_t *src)
{
  dst->type = HB_SERIAL_MSG_ARM_STATUS;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->armed,  sizeof(src->armed));
  hb_serial_finalize_message(dst);
}

inline void hb_serial_arm_status_msg_unpack(hb_serial_arm_status_msg_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->armed, src->payload + offset, sizeof(dst->armed));
}

inline size_t hb_serial_arm_status_msg_send_to_buffer(uint8_t *dst, const hb_serial_arm_status_msg_t *src)
{
  hb_serial_message_t msg;
  hb_serial_arm_status_msg_pack(&msg, src);
  return hb_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// zero message
//==============================================================================

inline void hb_serial_zero_msg_pack(hb_serial_message_t *dst, const hb_serial_zero_msg_t *src)
{
  dst->type = HB_SERIAL_MSG_ZERO;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->success,  sizeof(src->success));
  hb_serial_finalize_message(dst);
}

inline void hb_serial_zero_msg_unpack(hb_serial_zero_msg_t *dst, const hb_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->success, src->payload + offset, sizeof(dst->success));
}

inline size_t hb_serial_zero_msg_send_to_buffer(uint8_t *dst, const hb_serial_zero_msg_t *src)
{
  hb_serial_message_t msg;
  hb_serial_zero_msg_pack(&msg, src);
  return hb_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// parser
//==============================================================================

typedef enum
{
  HB_SERIAL_PARSE_STATE_IDLE,
  HB_SERIAL_PARSE_STATE_GOT_MAGIC,
  HB_SERIAL_PARSE_STATE_GOT_TYPE,
  HB_SERIAL_PARSE_STATE_GOT_PAYLOAD
} hb_serial_parse_state_t;

inline bool hb_serial_parse_byte(uint8_t byte, hb_serial_message_t *msg)
{
  static hb_serial_parse_state_t parse_state = HB_SERIAL_PARSE_STATE_IDLE;
  static uint8_t crc_value = 0;
  static size_t payload_index = 0;
  static hb_serial_message_t msg_buffer;

  bool got_message = false;
  switch (parse_state)
  {
  case HB_SERIAL_PARSE_STATE_IDLE:
    if (byte == HB_SERIAL_MAGIC)
    {
      crc_value = 0;
      payload_index = 0;

      msg_buffer.magic = byte;
      crc_value = hb_serial_update_crc(crc_value, byte);

      parse_state = HB_SERIAL_PARSE_STATE_GOT_MAGIC;
    }
    break;

  case HB_SERIAL_PARSE_STATE_GOT_MAGIC:
    msg_buffer.type = byte;
    crc_value = hb_serial_update_crc(crc_value, byte);
    parse_state = HB_SERIAL_PARSE_STATE_GOT_TYPE;
    break;

  case HB_SERIAL_PARSE_STATE_GOT_TYPE:
    msg_buffer.payload[payload_index++] = byte;
    crc_value = hb_serial_update_crc(crc_value, byte);
    if (payload_index == HB_SERIAL_PAYLOAD_LEN[msg_buffer.type])
    {
      parse_state = HB_SERIAL_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case HB_SERIAL_PARSE_STATE_GOT_PAYLOAD:
    msg_buffer.crc = byte;
    if (msg_buffer.crc == crc_value)
    {
      got_message = true;
      memcpy(msg, &msg_buffer, sizeof(msg_buffer));
    }
    parse_state = HB_SERIAL_PARSE_STATE_IDLE;
    break;
  }

  return got_message;
}

#endif // HB_SERIAL_H
