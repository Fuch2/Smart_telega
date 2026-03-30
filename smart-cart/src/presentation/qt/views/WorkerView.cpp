#include "WorkerView.hpp"
#include "../widgets/StatusPanelWidget.hpp"
#include "../widgets/SlotGridWidget.hpp"
#include "../viewmodels/WorkerViewModel.hpp"
#include "../../../infrastructure/hw/stm32/UartStm32Link.hpp"

#include <QVBoxLayout>
#include <QLabel>

WorkerView::WorkerView(QWidget* parent) : QWidget(parent) {
    buildUi();
    bindViewModel();
    vm_->start();
}

void WorkerView::buildUi() {
    auto* root  = new QVBoxLayout(this);
    auto* title = new QLabel(QString::fromUtf8("WorkerView: рабочий контур"), this);
    status_ = new StatusPanelWidget(this);
    grid_   = new SlotGridWidget(this);

    root->addWidget(title);
    root->addWidget(status_);
    root->addWidget(grid_, 1);
}

void WorkerView::bindViewModel() {
    vm_ = new WorkerViewModel(this);

    // --- реальный UART ---
    auto* uart = new smartcart::infrastructure::hw::stm32::UartStm32Link(
        "/dev/ttyAMA0",   // ← замени на свой порт
        115200,
        500
    );
    if (uart->open()) {
        vm_->setStm32Link(uart);
    }
    // ---------------------

    connect(vm_, &WorkerViewModel::rfidStatusChanged,
            status_, &StatusPanelWidget::setRfidStatus);
    connect(vm_, &WorkerViewModel::uartStatusChanged,
            status_, &StatusPanelWidget::setUartStatus);
    connect(vm_, &WorkerViewModel::scannerStatusChanged,
            status_, &StatusPanelWidget::setScannerStatus);
    connect(vm_, &WorkerViewModel::moduleSerialChanged,
            status_, &StatusPanelWidget::setModuleSerial);

    connect(vm_, &WorkerViewModel::slotOccupiedChanged,
            grid_, &SlotGridWidget::setSlotOccupied);
    connect(vm_, &WorkerViewModel::targetSlotChanged,
            grid_, &SlotGridWidget::setTargetSlot);

    connect(grid_, &SlotGridWidget::slotClicked,
            grid_, &SlotGridWidget::setTargetSlot);
}
