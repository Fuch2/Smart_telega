#include "WorkerViewModel.hpp"

// Полные include только здесь — MOC их не видит, это нормально
#include "application/ports/IStm32Link.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <QMetaObject>
#include <QString>

WorkerViewModel::WorkerViewModel(QObject* parent) : QObject(parent) {
    occupied_.fill(false);
}

void WorkerViewModel::setStm32Link(smartcart::application::ports::IStm32Link* link) {
    stm32_ = link;
    stm32_->setEventCallback([this](const smartcart::infrastructure::hw::stm32::Frame& frame) {
        QMetaObject::invokeMethod(this, [this, frame]() {
            onStm32Event(frame);
        }, Qt::QueuedConnection);
    });
}

void WorkerViewModel::start() {
    emit moduleSerialChanged("SN-DEMO-001");
    emit rfidStatusChanged("OK");
    emit uartStatusChanged("OK");
    emit scannerStatusChanged("READY");

    target_ = 7;
    emit targetSlotChanged(target_);

    emit slotOccupiedChanged(3, true);
    emit slotOccupiedChanged(11, true);
    emit slotOccupiedChanged(18, true);
    occupied_[2]  = true;
    occupied_[10] = true;
    occupied_[17] = true;
}

void WorkerViewModel::stop() {
    if (stm32_) stm32_->close();
}

void WorkerViewModel::onStm32Event(const smartcart::infrastructure::hw::stm32::Frame& frame) {
    using namespace smartcart::infrastructure::hw::stm32;

    if (frame.frameType != FrameType::Evt) return;

    const auto cmd = static_cast<CommandId>(frame.commandId);

    if (cmd == CommandId::EvtSwitchChanged) {
        if (frame.payload.size() < 2) return;
        const int  slot     = frame.payload[0];
        const bool occupied = frame.payload[1] != 0x00;
        if (slot < 1 || slot > 24) return;
        occupied_[slot - 1] = occupied;
        emit slotOccupiedChanged(slot, occupied);
    }
    else if (cmd == CommandId::EvtReady) {
        emit uartStatusChanged("OK");
    }
    else if (cmd == CommandId::EvtFault) {
        emit uartStatusChanged("FAULT");
    }
}
