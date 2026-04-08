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
    stack_ = new QStackedWidget(central);

    switchingPage_ = new QWidget(central);
    switchingPage_->setStyleSheet("QWidget { background: #102217; }");
    auto* switchingLayout = new QVBoxLayout(switchingPage_);
    switchingLayout->setContentsMargins(64, 56, 64, 56);
    switchingLayout->addStretch();

    auto* switchingCard = new QFrame(switchingPage_);
    switchingCard->setFrameShape(QFrame::StyledPanel);
    switchingCard->setMinimumSize(760, 260);
    switchingCard->setMaximumWidth(920);
    switchingCard->setStyleSheet(
        "QFrame { background: #F6FAF7; border: 1px solid #AFC8B8; "
        "border-radius: 8px; }");

    auto* switchingCardLayout = new QVBoxLayout(switchingCard);
    switchingCardLayout->setContentsMargins(40, 36, 40, 36);
    switchingCardLayout->setSpacing(18);

    auto* switchingTitle = new QLabel(QString::fromUtf8("Переключаем модуль"), switchingCard);
    switchingTitle->setFont(QFont("Segoe UI", 30, QFont::Bold));
    switchingTitle->setStyleSheet("color: #102217;");
    switchingTitle->setAlignment(Qt::AlignCenter);

    auto* switchingText = new QLabel(
        QString::fromUtf8("Подождите, данные нового модуля загружаются..."),
        switchingCard);
    switchingText->setFont(QFont("Segoe UI", 18));
    switchingText->setStyleSheet("color: #31533D;");
    switchingText->setAlignment(Qt::AlignCenter);
    switchingText->setWordWrap(true);

    switchingCardLayout->addWidget(switchingTitle);
    switchingCardLayout->addWidget(switchingText);
    switchingLayout->addWidget(switchingCard, 0, Qt::AlignCenter);
    switchingLayout->addStretch();

    workerView_ = new WorkerView(*workerVm, central);
    adminView_  = new AdminView(adminVm,    central);

    stack_->addWidget(workerView_);
    stack_->addWidget(adminView_);
    stack_->addWidget(switchingPage_);
    stack_->setCurrentWidget(workerView_);

    root->addWidget(sideRail);
    root->addWidget(stack_, 1);
    setCentralWidget(central);
}

void MainWindow::wireSignals() {
    connect(workerBtn_, &QPushButton::clicked, this, [this]() {
        if (workerView_) {
            stack_->setCurrentWidget(workerView_);
        }
    });
    connect(adminBtn_, &QPushButton::clicked, this, [this]() {
        if (adminView_) {
            stack_->setCurrentWidget(adminView_);
        }
    });
}

void MainWindow::beginModuleSwitch() {
    restoreAdminPageAfterSwitch_ =
        stack_ && adminView_ && stack_->currentWidget() == adminView_;

    workerBtn_->setEnabled(false);
    adminBtn_->setEnabled(false);

    if (workerView_) {
        stack_->removeWidget(workerView_);
        delete workerView_;
        workerView_ = nullptr;
    }
    if (adminView_) {
        stack_->removeWidget(adminView_);
        delete adminView_;
        adminView_ = nullptr;
    }

    if (switchingPage_) {
        stack_->setCurrentWidget(switchingPage_);
    }
}

void MainWindow::finishModuleSwitch(AdminViewModel* adminVm,
                                    WorkerViewModel* workerVm) {
    workerView_ = new WorkerView(*workerVm, centralWidget());
    adminView_  = new AdminView(adminVm, centralWidget());

    stack_->insertWidget(0, workerView_);
    stack_->insertWidget(1, adminView_);
    stack_->setCurrentWidget(restoreAdminPageAfterSwitch_
                                 ? static_cast<QWidget*>(adminView_)
                                 : static_cast<QWidget*>(workerView_));

    workerBtn_->setEnabled(true);
    adminBtn_->setEnabled(true);
}
