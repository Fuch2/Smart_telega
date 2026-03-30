#include "MainWindow.hpp"
#include "views/WorkerView.hpp"
#include "views/AdminView.hpp"
#include "viewmodels/AdminViewModel.hpp"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QStackedWidget>

MainWindow::MainWindow(AdminViewModel* adminVm, QWidget* parent)
    : QMainWindow(parent) {
    buildUi(adminVm);
    wireSignals();
    resize(1600, 900);
}

void MainWindow::buildUi(AdminViewModel* adminVm) {
    auto* central = new QWidget(this);
    auto* root    = new QVBoxLayout(central);

    auto* topBar  = new QHBoxLayout();
    workerBtn_ = new QPushButton(QString::fromUtf8("Рабочий режим"), central);
    adminBtn_  = new QPushButton(QString::fromUtf8("Админ"), central);
    workerBtn_->setMinimumHeight(64);
    adminBtn_->setMinimumHeight(64);
    topBar->addWidget(workerBtn_);
    topBar->addWidget(adminBtn_);

    stack_      = new QStackedWidget(central);
    workerView_ = new WorkerView(central);
    adminView_  = new AdminView(adminVm, central);  // DI: vm приходит снаружи

    stack_->addWidget(workerView_);
    stack_->addWidget(adminView_);
    stack_->setCurrentWidget(workerView_);

    root->addLayout(topBar);
    root->addWidget(stack_);
    setCentralWidget(central);
}

void MainWindow::wireSignals() {
    connect(workerBtn_, &QPushButton::clicked, this, [this]() {
        stack_->setCurrentWidget(workerView_);
    });
    connect(adminBtn_, &QPushButton::clicked, this, [this]() {
        stack_->setCurrentWidget(adminView_);
    });
}
