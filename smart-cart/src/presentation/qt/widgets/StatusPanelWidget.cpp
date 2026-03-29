#include "StatusPanelWidget.hpp"
#include <QGridLayout>
#include <QLabel>

StatusPanelWidget::StatusPanelWidget(QWidget* parent) : QWidget(parent) {
    auto* g = new QGridLayout(this);

    g->addWidget(new QLabel("RFID:", this), 0, 0);
    rfid_ = new QLabel("UNKNOWN", this);
    g->addWidget(rfid_, 0, 1);

    g->addWidget(new QLabel("UART:", this), 0, 2);
    uart_ = new QLabel("UNKNOWN", this);
    g->addWidget(uart_, 0, 3);

    g->addWidget(new QLabel("SCANNER:", this), 1, 0);
    scanner_ = new QLabel("UNKNOWN", this);
    g->addWidget(scanner_, 1, 1);

    g->addWidget(new QLabel("MODULE SN:", this), 1, 2);
    serial_ = new QLabel("-", this);
    g->addWidget(serial_, 1, 3);
}

void StatusPanelWidget::setRfidStatus(const QString& v)   { rfid_->setText(v); }
void StatusPanelWidget::setUartStatus(const QString& v)   { uart_->setText(v); }
void StatusPanelWidget::setScannerStatus(const QString& v){ scanner_->setText(v); }
void StatusPanelWidget::setModuleSerial(const QString& v) { serial_->setText(v); }
