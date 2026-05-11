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

    g->addWidget(new QLabel(QString::fromUtf8("АКБ:"), this), 2, 0);
    battery_ = new QLabel(QString::fromUtf8("N/A"), this);
    g->addWidget(battery_, 2, 1, 1, 3); // занимает 3 колонки
}

void StatusPanelWidget::setRfidStatus(const QString& v)   { rfid_->setText(v); }
void StatusPanelWidget::setUartStatus(const QString& v)   { uart_->setText(v); }
void StatusPanelWidget::setScannerStatus(const QString& v){ scanner_->setText(v); }
void StatusPanelWidget::setModuleSerial(const QString& v) { serial_->setText(v); }

void StatusPanelWidget::setBatteryLevel(int level) {
    if (level < 0) {
        battery_->setText(QString::fromUtf8("N/A"));
        battery_->setStyleSheet({});
        return;
    }

    // Иконка + процент
    const QString icon =
        level >= 75 ? QString::fromUtf8("🔋") :
        level >= 40 ? QString::fromUtf8("🪫") :
                      QString::fromUtf8("⚠️");
    battery_->setText(QString::fromUtf8("%1 %2%").arg(icon).arg(level));

    // Цвет по уровню заряда
    if (level >= 50) {
        battery_->setStyleSheet("color: #a6e3a1;"); // зелёный
    } else if (level >= 20) {
        battery_->setStyleSheet("color: #f9e2af;"); // жёлтый
    } else {
        battery_->setStyleSheet("color: #f38ba8; font-weight: bold;"); // красный
    }
}
