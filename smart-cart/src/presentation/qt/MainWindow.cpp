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
#include <QGridLayout>
#include <QPushButton>
#include <QStackedWidget>
#include <QFrame>
#include <QLabel>
#include <QFont>

MainWindow::MainWindow(AdminViewModel*  adminVm,
                       WorkerViewModel* workerVm,
                       smartcart::infrastructure::hw::stm32::MockStm32Link* mock,
                       QWidget* parent)
    : QMainWindow(parent)
{
    buildUi(adminVm, workerVm, mock);
    wireSignals();
    resize(1280, 800);
}

void MainWindow::buildUi(AdminViewModel* adminVm, WorkerViewModel* workerVm,
                         smartcart::infrastructure::hw::stm32::MockStm32Link* mock)
{
    auto* central = new QWidget(this);
    central->setStyleSheet(
        "QWidget { background: #102217; color: #102217; }"
        "QPushButton { background: #E6F0E9; color: #173025; "
        "border: 1px solid #B8D0BF; border-radius: 8px; "
        "padding: 12px 14px; font-weight: bold; }"
        "QPushButton:hover { background: #D7E8DC; }"
        "QPushButton:pressed { background: #C4DACB; }"
    );
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // ── Боковая навигация: крупные зоны для сенсорного экрана ───────────────
    auto* sideRail = new QFrame(central);
    sideRail->setFixedWidth(230);
    sideRail->setStyleSheet(
        "QFrame { background: #0B1A12; border-right: 1px solid #31533D; }"
    );

    auto* sideLayout = new QVBoxLayout(sideRail);
    sideLayout->setContentsMargins(16, 20, 16, 20);
    sideLayout->setSpacing(14);

    auto* title = new QLabel(QString::fromUtf8("SmartCart"), sideRail);
    title->setFont(QFont("Segoe UI", 24, QFont::Bold));
    title->setStyleSheet("color: #FFFFFF;");

    auto* subtitle = new QLabel(QString::fromUtf8("Пульт тележки"), sideRail);
    subtitle->setFont(QFont("Segoe UI", 12, QFont::Bold));
    subtitle->setStyleSheet("color: #8DE4AD;");

    workerBtn_ = new QPushButton(QString::fromUtf8("Рабочий режим"), central);
    adminBtn_  = new QPushButton(QString::fromUtf8("Админ"),         central);
    workerBtn_->setMinimumHeight(72);
    adminBtn_->setMinimumHeight(72);
    workerBtn_->setFont(QFont("Segoe UI", 14, QFont::Bold));
    adminBtn_->setFont(QFont("Segoe UI", 14, QFont::Bold));
    workerBtn_->setStyleSheet(
        "QPushButton { background: #2F8F57; color: #FFFFFF; "
        "border: 1px solid #2F8F57; border-radius: 8px; }"
        "QPushButton:hover { background: #3CA86A; }"
        "QPushButton:pressed { background: #267346; }"
    );
    adminBtn_->setStyleSheet(
        "QPushButton { background: #183928; color: #DDEBE1; "
        "border: 1px solid #3E6B4C; border-radius: 8px; }"
        "QPushButton:hover { background: #214B34; }"
        "QPushButton:pressed { background: #122A1E; }"
    );

    sideLayout->addWidget(title);
    sideLayout->addWidget(subtitle);
    sideLayout->addSpacing(14);
    sideLayout->addWidget(workerBtn_);
    sideLayout->addWidget(adminBtn_);

    // ── Тестовая панель кнопок PA1/PA2 (только в demoMode) ────────────────────
    if (mock) {
        auto* demoFrame = new QFrame(central);
        demoFrame->setFrameShape(QFrame::StyledPanel);
        demoFrame->setStyleSheet(
            "QFrame { background: #122A1E; border: 1px solid #31533D; "
            "border-radius: 8px; }");

        auto* demoLayout = new QVBoxLayout(demoFrame);
        demoLayout->setContentsMargins(10, 10, 10, 10);
        demoLayout->setSpacing(8);

        auto* demoLabel = new QLabel(QString::fromUtf8("Demo PA1 / PA2"), demoFrame);
        demoLabel->setFont(QFont("Segoe UI", 11, QFont::Bold));
        demoLabel->setStyleSheet("color: #8DE4AD;");
        demoLayout->addWidget(demoLabel);

        auto makeDemoButton = [demoFrame](const QString& text,
                                          const QString& color,
                                          const QString& pressed) {
            auto* button = new QPushButton(text, demoFrame);
            button->setMinimumHeight(46);
            button->setFont(QFont("Segoe UI", 10, QFont::Bold));
            button->setStyleSheet(QString(
                "QPushButton { background: %1; color: #FFFFFF; "
                "border: 1px solid %1; border-radius: 8px; padding: 8px; }"
                "QPushButton:pressed { background: %2; }"
            ).arg(color, pressed));
            return button;
        };

        // PA1 — вставить катушку (слот 1)
        auto* pa1InsertBtn = makeDemoButton(
            QString::fromUtf8("PA1: вставить"), "#2F8F57", "#267346");

        // PA1 — вынуть катушку (слот 1)
        auto* pa1RemoveBtn = makeDemoButton(
            QString::fromUtf8("PA1: вынуть"), "#B85C5C", "#954747");

        // PA2 — вставить катушку (слот 2)
        auto* pa2InsertBtn = makeDemoButton(
            QString::fromUtf8("PA2: вставить"), "#2F8F57", "#267346");

        // PA2 — вынуть катушку (слот 2)
        auto* pa2RemoveBtn = makeDemoButton(
            QString::fromUtf8("PA2: вынуть"), "#B85C5C", "#954747");

        demoLayout->addWidget(pa1InsertBtn);
        demoLayout->addWidget(pa1RemoveBtn);
        demoLayout->addWidget(pa2InsertBtn);
        demoLayout->addWidget(pa2RemoveBtn);

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

        sideLayout->addWidget(demoFrame);
    }

    sideLayout->addStretch();

    // ── Стек экранов ───────────────────────────────────────────────────────────
    stack_      = new QStackedWidget(central);
    workerView_ = new WorkerView(*workerVm, central);
    adminView_  = new AdminView(adminVm,    central);

    stack_->addWidget(workerView_);
    stack_->addWidget(adminView_);
    stack_->setCurrentWidget(workerView_);

    root->addWidget(sideRail);
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
