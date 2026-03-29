#include "WorkerViewModel.hpp"
#include <QTimer>
#include <QString>

#include "../../../infrastructure/persistence/SqliteModuleRepository.hpp"
#include <QCoreApplication>

WorkerViewModel::WorkerViewModel(QObject* parent) : QObject(parent) {
    occupied_.fill(false);

    timer_ = new QTimer(this);
    timer_->setInterval(900); // ~1 Hz
    connect(timer_, &QTimer::timeout, this, &WorkerViewModel::onTick);
}

void WorkerViewModel::start() {
    emit moduleSerialChanged("SN-DEMO-001");
    emit rfidStatusChanged("OK");
    emit uartStatusChanged("OK");
    emit scannerStatusChanged("READY");

    // начальная картинка
    target_ = 7;
    emit targetSlotChanged(target_);
    emit slotOccupiedChanged(3, true);
    emit slotOccupiedChanged(11, true);
    emit slotOccupiedChanged(18, true);
    occupied_[2] = true;
    occupied_[10] = true;
    occupied_[17] = true;

    timer_->start();
}

void WorkerViewModel::stop() {
    timer_->stop();
}

int WorkerViewModel::wrap1to24(int v) {
    while (v < 1) v += 24;
    while (v > 24) v -= 24;
    return v;
}

void WorkerViewModel::onTick() {
    ++tick_;

    // 1) мигание scanner статуса
    emit scannerStatusChanged((tick_ % 2 == 0) ? "READY" : "BUSY");

    // 2) target двигаем по кругу
    target_ = wrap1to24(target_ + 1);
    emit targetSlotChanged(target_);

    // 3) раз в 2 тика переключаем занятость одного слота
    if (tick_ % 2 == 0) {
        int slot = wrap1to24((tick_ / 2) % 24 + 1);
        bool newVal = !occupied_[slot - 1];
        occupied_[slot - 1] = newVal;
        emit slotOccupiedChanged(slot, newVal);
    }

    // 4) иногда “деградация” UART/RFID для демонстрации
    if (tick_ % 10 == 0) {
        emit uartStatusChanged("WARN");
    } else if (tick_ % 10 == 2) {
        emit uartStatusChanged("OK");
    }

    if (tick_ % 14 == 0) {
        emit rfidStatusChanged("RETRY");
    } else if (tick_ % 14 == 3) {
        emit rfidStatusChanged("OK");
    }
}
