#pragma once
#include <QWidget>

class QLabel;

class StatusPanelWidget final : public QWidget {
    Q_OBJECT
public:
    explicit StatusPanelWidget(QWidget* parent = nullptr);

    void setRfidStatus(const QString& v);
    void setUartStatus(const QString& v);
    void setScannerStatus(const QString& v);
    void setModuleSerial(const QString& v);

private:
    QLabel* rfid_{nullptr};
    QLabel* uart_{nullptr};
    QLabel* scanner_{nullptr};
    QLabel* serial_{nullptr};
};
