// ===== src/presentation/qt/views/ErrorDialog.cpp =====
#include "ErrorDialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFont>

ErrorDialog::ErrorDialog(const QString& title,
                         const QString& message,
                         QWidget*       parent)
    : QDialog(parent)
{
    setWindowTitle(title);
    setModal(true);
    setMinimumWidth(400);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 20, 20, 20);
    root->setSpacing(12);

    auto* icon = new QLabel("⚠", this);
    icon->setFont(QFont("Segoe UI", 32));
    icon->setAlignment(Qt::AlignCenter);
    icon->setStyleSheet("color: #F38BA8;");
    root->addWidget(icon);

    auto* titleLabel = new QLabel(title, this);
    titleLabel->setFont(QFont("Segoe UI", 13, QFont::Bold));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("color: #CDD6F4;");
    root->addWidget(titleLabel);

    auto* msgLabel = new QLabel(message, this);
    msgLabel->setFont(QFont("Segoe UI", 10));
    msgLabel->setWordWrap(true);
    msgLabel->setAlignment(Qt::AlignCenter);
    msgLabel->setStyleSheet("color: #A6ADC8;");
    root->addWidget(msgLabel);

    auto* btnRow = new QHBoxLayout();
    auto* okBtn  = new QPushButton("OK", this);
    okBtn->setMinimumWidth(80);
    okBtn->setStyleSheet(
        "QPushButton { background: #89B4FA; color: #1E1E2E; "
        "border-radius: 4px; padding: 6px 20px; font-weight: bold; }"
        "QPushButton:hover { background: #B4BEFE; }"
    );
    btnRow->addStretch();
    btnRow->addWidget(okBtn);
    btnRow->addStretch();
    root->addLayout(btnRow);

    connect(okBtn, &QPushButton::clicked, this, &QDialog::accept);

    setStyleSheet("ErrorDialog { background: #181825; }");
}

void ErrorDialog::show(const QString& title,
                       const QString& message,
                       QWidget*       parent)
{
    ErrorDialog dlg(title, message, parent);
    dlg.exec();
}
