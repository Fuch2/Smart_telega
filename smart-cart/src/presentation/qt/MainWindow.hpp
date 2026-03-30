// ===== src/presentation/qt/MainWindow.hpp =====
// Исправлено: файл был пустым (содержал .cpp код) — создан настоящий .hpp
#pragma once

#include <QMainWindow>

class QStackedWidget;
class QPushButton;
class WorkerView;
class AdminView;
class AdminViewModel;
class WorkerViewModel;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(AdminViewModel*  adminVm,
                        WorkerViewModel* workerVm,
                        QWidget*         parent = nullptr);

private:
    QStackedWidget* stack_      = nullptr;
    WorkerView*     workerView_ = nullptr;
    AdminView*      adminView_  = nullptr;
    QPushButton*    workerBtn_  = nullptr;
    QPushButton*    adminBtn_   = nullptr;

    void buildUi(AdminViewModel* adminVm, WorkerViewModel* workerVm);
    void wireSignals();
};
