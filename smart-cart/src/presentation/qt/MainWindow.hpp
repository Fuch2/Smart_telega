// ===== src/presentation/qt/MainWindow.hpp =====
#pragma once

#include <QMainWindow>

class QStackedWidget;
class QPushButton;
class WorkerView;
class AdminView;
class AdminViewModel;
class WorkerViewModel;

namespace smartcart::infrastructure::hw::stm32 {
    class MockStm32Link;
}

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(AdminViewModel*  adminVm,
                        WorkerViewModel* workerVm,
                        // nullptr в prod-режиме, mock в demoMode
                        smartcart::infrastructure::hw::stm32::MockStm32Link* mock = nullptr,
                        QWidget* parent = nullptr);

private:
    QStackedWidget* stack_      = nullptr;
    WorkerView*     workerView_ = nullptr;
    AdminView*      adminView_  = nullptr;
    QPushButton*    workerBtn_  = nullptr;
    QPushButton*    adminBtn_   = nullptr;

    void buildUi(AdminViewModel* adminVm, WorkerViewModel* workerVm,
                 smartcart::infrastructure::hw::stm32::MockStm32Link* mock);
    void wireSignals();
};
