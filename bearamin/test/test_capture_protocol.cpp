#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../firmware/common/capture_protocol.h"

using namespace bearamin_capture;

static void testRecordRoundTripAndCrcRejection() {
  uint8_t payload[] = {1, 2, 3, 4, 5};
  uint8_t encoded[kMaxRecordSize] = {};
  const size_t size = encodeRecord(
      RECORD_DIAGNOSTIC,
      FLAG_PARTIAL,
      0xFFFFFFFEu,
      0x0102030405060708ULL,
      payload,
      sizeof(payload),
      encoded,
      sizeof(encoded));
  assert(size == kRecordOverhead + sizeof(payload));

  RecordView record = {};
  size_t consumed = 0;
  assert(decodeRecord(encoded, size, record, &consumed));
  assert(consumed == size);
  assert(record.type == RECORD_DIAGNOSTIC);
  assert(record.flags == FLAG_PARTIAL);
  assert(record.sequence == 0xFFFFFFFEu);
  assert(record.coreUs == 0x0102030405060708ULL);
  assert(record.payloadSize == sizeof(payload));
  assert(memcmp(record.payload, payload, sizeof(payload)) == 0);

  encoded[kHeaderSize + 2] ^= 0x80u;
  assert(!decodeRecord(encoded, size, record));
}

static void testPartialStreamParserAndSequenceRollover() {
  uint8_t first[kMaxRecordSize] = {};
  uint8_t second[kMaxRecordSize] = {};
  const uint8_t payload = 0x55;
  const size_t firstSize = encodeRecord(
      RECORD_RESERVED_2,
      FLAG_NONE,
      0xFFFFFFFFu,
      100,
      &payload,
      1,
      first,
      sizeof(first));
  const size_t secondSize = encodeRecord(
      RECORD_RESERVED_2,
      FLAG_NONE,
      0,
      200,
      &payload,
      1,
      second,
      sizeof(second));

  StreamParser<> parser;
  RecordView record = {};
  uint8_t decoded = 0;
  for (size_t i = 0; i < firstSize; ++i) {
    if (parser.Push(first[i], record)) {
      assert(record.sequence == 0xFFFFFFFFu);
      ++decoded;
    }
  }
  for (size_t i = 0; i < secondSize; ++i) {
    if (parser.Push(second[i], record)) {
      assert(record.sequence == 0);
      ++decoded;
    }
  }
  assert(decoded == 2);
  assert(parser.CrcErrors() == 0);
}

static void testByteRingWrapAndNonBlockingFailure() {
  ByteRing<16> ring;
  const uint8_t first[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  assert(ring.TryPush(first, sizeof(first)));
  assert(ring.Used() == sizeof(first));
  ring.Consume(7);

  const uint8_t second[] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
  assert(ring.TryPush(second, sizeof(second)));
  assert(ring.HighWater() == 13);
  const uint8_t tooLarge[] = {20, 21, 22, 23};
  assert(!ring.TryPush(tooLarge, sizeof(tooLarge)));

  uint8_t output[16] = {};
  const size_t count = ring.Peek(output, sizeof(output));
  assert(count == 13);
  const uint8_t expected[] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
  assert(memcmp(output, expected, sizeof(expected)) == 0);
}

static void testProcessedSnapshotContract() {
  ProcessedSensorSnapshotV1 source = {};
  source.sensorFlags = SENSOR_MATRIX_READY | SENSOR_IMU_READY;
  source.reserved0 = 27;
  source.reserved1 = 4095;
  source.matrixZMm = 81;
  source.matrixXMilli = 123;
  source.matrixYMilli = 876;
  source.matrixZNorm = 3000;
  source.matrixXNorm = 700;
  source.matrixYNorm = 3500;
  source.matrixValidCount = 42;
  source.matrixBlobCount = 11;
  source.impactVelocityNorm = 2048;
  source.impactAgeMs = 7;
  source.accelXMilliG = -120;
  source.accelYMilliG = 40;
  source.accelZMilliG = 1010;
  source.sensorAgeMs = 3;

  uint8_t payload[kProcessedSnapshotV1PayloadSize] = {};
  encodeProcessedSnapshotV1(source, payload);
  ProcessedSensorSnapshotV1 decoded = {};
  assert(decodeProcessedSnapshotV1(payload, sizeof(payload), decoded));
  assert(decoded.sensorFlags == source.sensorFlags);
  assert(decoded.reserved0 == source.reserved0);
  assert(decoded.matrixXMilli == source.matrixXMilli);
  assert(decoded.matrixYMilli == source.matrixYMilli);
  assert(decoded.impactVelocityNorm == source.impactVelocityNorm);
  assert(decoded.accelXMilliG == source.accelXMilliG);
  assert(decoded.sensorAgeMs == source.sensorAgeMs);

  ProcessedSensorSnapshotV2 sourceV2 = {};
  static_cast<ProcessedSensorSnapshotV1 &>(sourceV2) = source;
  sourceV2.sensorFlags |= SENSOR_SPANK_GATE;
  sourceV2.pitchNorm = 1234;
  sourceV2.rollNorm = 3456;
  sourceV2.orientationConfidence = 219;
  uint8_t payloadV2[kProcessedSnapshotV2PayloadSize] = {};
  encodeProcessedSnapshotV2(sourceV2, payloadV2);
  ProcessedSensorSnapshotV2 decodedV2 = {};
  assert(decodeProcessedSnapshotV2(payloadV2, sizeof(payloadV2), decodedV2));
  assert(decodedV2.sensorFlags == sourceV2.sensorFlags);
  assert(decodedV2.matrixZMm == source.matrixZMm);
  assert(decodedV2.pitchNorm == sourceV2.pitchNorm);
  assert(decodedV2.rollNorm == sourceV2.rollNorm);
  assert(decodedV2.orientationConfidence == sourceV2.orientationConfidence);
  assert(!decodeProcessedSnapshotV1(payloadV2, sizeof(payloadV2), source));
}

int main() {
  testRecordRoundTripAndCrcRejection();
  testPartialStreamParserAndSequenceRollover();
  testByteRingWrapAndNonBlockingFailure();
  testProcessedSnapshotContract();
  return 0;
}
