// ===== src/presentation/qt/MainWindow.cpp =====
#include "MainWindow.hpp"
#include "views/WorkerView.hpp"
#include "views/AdminView.hpp"
#include "viewmodels/AdminViewModel.hpp"
#include "viewmodels/WorkerViewModel.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QStackedWidget>
#include <QFrame>
#include <QLabel>

MainWindow::MainWindow(AdminViewModel*  adminVm,
                       WorkerViewModel* workerVm,
                       smartcart::infrastructure::hw::stm32::MockStm32Link* mock,
                       QWidget* parent)
    : QMainWindow(parent)
{
    buildUi(adminVm, workerVm, mock);
    wireSignals();
    resize(1600, 900);
}

void MainWindow::buildUi(AdminViewModel* adminVm, WorkerViewModel* workerVm,
                         smartcart::infrastructure::hw::stm32::MockStm32Link* mock)
{
    auto* central = new QWidget(this);
    central->setStyleSheet(
        "QWidget { background: #F0F4F1; color: #223027; }"
        "QPushButton { background: #E3ECE5; color: #223027; "
        "border: 1px solid #C5D2C8; border-radius: 4px; "
        "padding: 8px 14px; font-weight: bold; }"
        "QPushButton:hover { background: #D6E5DA; }"
        "QPushButton:pressed { background: #C3D7C9; }"
    );
    auto* root    = new QVBoxLayout(central);

    // ── Верхняя панель навигации ───────────────────────────────────────────────
    auto* topBar = new QHBoxLayout();
    workerBtn_ = new QPushButton(QString::fromUtf8("Рабочий режим"), central);
    adminBtn_  = new QPushButton(QString::fromUtf8("Админ"),         central);
    workerBtn_->setMinimumHeight(48);
    adminBtn_->setMinimumHeight(48);
    topBar->addWidget(workerBtn_);
    topBar->addWidget(adminBtn_);
    topBar->addStretch();

    // ── Тестовая панель кнопок PA1/PA2 (только в demoMode) ────────────────────
    if (mock) {
        auto* demoFrame = new QFrame(central);
        demoFrame->setFrameShape(QFrame::StyledPanel);
        demoFrame->setStyleSheet(
            "QFrame { background: #FFFFFF; border: 1px solid #D8E1D9; "
            "border-radius: 6px; padding: 4px; }");

        auto* demoLayout = new QHBoxLayout(demoFrame);
        demoLayout->setContentsMargins(8, 4, 8, 4);
        demoLayout->setSpacing(8);

        auto* demoLabel = new QLabel(QString::fromUtf8("🧪 Demo:"), demoFrame);
        demoLabel->setStyleSheet("color: #6E6A3A; font-weight: bold;");
        demoLayout->addWidget(demoLabel);

        // PA1 — вставить катушку (слот 1)
        auto* pa1InsertBtn = new QPushButton(
            QString::fromUtf8("PA1: вставить (слот 1)"), demoFrame);
        pa1InsertBtn->setStyleSheet(
            "QPushButton { background: #2E7D4F; color: #FFFFFF; "
            "border-radius: 4px; padding: 4px 12px; font-weight: bold; }"
            "QPushButton:pressed { background: #24663F; }");

        // PA1 — вынуть катушку (слот 1)
        auto* pa1RemoveBtn = new QPushButton(
            QString::fromUtf8("PA1: вынуть (слот 1)"), demoFrame);
        pa1RemoveBtn->setStyleSheet(
            "QPushButton { background: #B85C5C; color: #FFFFFF; "
            "border-radius: 4px; padding: 4px 12px; font-weight: bold; }"
            "QPushButton:pressed { background: #954747; }");

        // PA2 — вставить катушку (слот 2)
        auto* pa2InsertBtn = new QPushButton(
            QString::fromUtf8("PA2: вставить (слот 2)"), demoFrame);
        pa2InsertBtn->setStyleSheet(
            "QPushButton { background: #4E8F61; color: #FFFFFF; "
            "border-radius: 4px; padding: 4px 12px; font-weight: bold; }"
            "QPushButton:pressed { background: #3E744E; }");

        // PA2 — вынуть катушку (слот 2)
        auto* pa2RemoveBtn = new QPushButton(
            QString::fromUtf8("PA2: вынуть (слот 2)"), demoFrame);
        pa2RemoveBtn->setStyleSheet(
            "QPushButton { background: #C47B5A; color: #FFFFFF; "
            "border-radius: 4px; padding: 4px 12px; font-weight: bold; }"
            "QPushButton:pressed { background: #9E6146; }");

        demoLayout->addWidget(pa1InsertBtn);
        demoLayout->addWidget(pa1RemoveBtn);
        demoLayout->addWidget(pa2InsertBtn);
        demoLayout->addWidget(pa2RemoveBtn);
        demoLayout->addStretch();

        // Подключаем кнопки к mock
        connect(pa1InsertBtn, &QPushButton::clicked, [mock]() {
            mock->simulateSwitchEvent(1, true);
        });
        connect(pa1RemoveBtn, &QPushButton::clicked, [mock]() {
            mock->simulateSwitchEvent(1, false);
        });
        connect(pa2InsertBtn, &QPushButton::clicked, [mock]() {
            mock->simulateSwitchEvent(3, true);
        });
        connect(pa2RemoveBtn, &QPushButton::clicked, [mock]() {
            mock->simulateSwitchEvent(3, false);
        });

        topBar->addWidget(demoFrame);
    }

    // ── Стек экранов ───────────────────────────────────────────────────────────
    stack_      = new QStackedWidget(central);
    workerView_ = new WorkerView(*workerVm, central);
    adminView_  = new AdminView(adminVm,    central);

    stack_->addWidget(workerView_);
    stack_->addWidget(adminView_);
    stack_->setCurrentWidget(workerView_);

    root->addLayout(topBar);
    root->addWidget(stack_, 1);
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
