#!/usr/bin/env python3
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MATRIXCORE = PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/BearaminMatrixCoreApp.cpp"
DPT = PROJECT_ROOT / "dpt" / "DPT.multimode" / "DPT.multimode.cpp"
SIDECAR = PROJECT_ROOT / "firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp"


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def main() -> None:
    matrix = MATRIXCORE.read_text()
    recovery = function_body(
        matrix, "void recoverOpticalIfNeeded(", "\nbearamin_bridge::Packet buildDptPacket("
    )
    worker = function_body(
        matrix, "void opticalRecoveryWorker(", "\nvoid startOpticalRecoveryWorker("
    )
    assert "xTaskNotifyGive(opticalRecoveryTaskHandle)" in recovery
    assert "initSharedOpticalBus(" not in recovery
    assert "recoverMatrixSensor(opticalRecoveryWorking)" in worker
    assert "initSharedOpticalBus(opticalRecoveryWorking, false)" in worker
    assert "void pollTof(" not in matrix
    assert "opticalRecoveryActive()" in function_body(
        matrix, "void pollMatrix(", "\nvoid recoverOpticalIfNeeded("
    )
    assert (
        'xTaskCreateStaticPinnedToCore(\n'
        '      opticalRecoveryWorker,\n'
        '      "optical_recovery",'
    ) in matrix

    dpt = DPT.read_text()
    parser = function_body(
        dpt, "void AcceptBearaminBridgeByte(", "\nvoid BearaminUartRxDmaCallback("
    )
    assert "bearamin_raw_packets.Publish(packet);" in parser
    assert "bearamin_packet         = packet;" not in parser
    callback = function_body(
        dpt, "void AudioCallback(", "\nvoid BearaminUartRxDmaCallback("
    )
    assert "bearamin_predictor.Process(" in callback
    assert "bearamin_trajectory_reconstructor.Process(" in callback
    assert "expf(" not in callback
    assert "bearamin_audio_packet =" in callback
    assert "bearamin_trajectory_reconstructor.Prepare(" in parser
    assert "PerformanceCycleCount() - callback_start" in dpt
    assert "bearamin_uart_tx_max_irq_off_cycles" in dpt
    dpt_tx = function_body(
        dpt, "bool DirectTransmitBearaminBytes(", "\nbool BearaminPresetTxQueueEmpty("
    )
    assert "bearamin_uart.BlockingTransmit(" in dpt_tx
    assert "ConfigureBearaminSingleWireTxPin" not in dpt_tx
    assert "WriteBearaminSingleWireLevel" not in dpt_tx
    assert "ScopedIrqBlocker" not in dpt_tx

    sidecar = SIDECAR.read_text()
    assert 'body += ",\\"dpt_audio_deadline_misses\\":";' in sidecar
    assert 'body += ",\\"dpt_return_tx_max_irq_off_us\\":";' in sidecar
    assert 'line.startsWith("DPTP,")' in sidecar


if __name__ == "__main__":
    main()
