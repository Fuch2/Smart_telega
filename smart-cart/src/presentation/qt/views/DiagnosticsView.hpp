// ===== src/presentation/qt/views/DiagnosticsView.hpp =====
#pragma once

#include <QWidget>

class QLabel;
class QPushButton;
class StatusPanelWidget;

class DiagnosticsView final : public QWidget {
    Q_OBJECT
public:
    explicit DiagnosticsView(QWidget* parent = nullptr);

    void setUartStatus(const QString& v);
    void setScannerStatus(const QString& v);
    void setRfidStatus(const QString& v);
    void setModuleSerial(const QString& v);
    void appendLog(const QString& line);

private:
    StatusPanelWidget* statusPanel_ = nullptr;
    QLabel*            logLabel_    = nullptr;
    QPushButton*       clearBtn_    = nullptr;

    QString logText_;
};
