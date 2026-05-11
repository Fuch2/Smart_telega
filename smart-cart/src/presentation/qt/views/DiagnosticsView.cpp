// ===== src/presentation/qt/views/DiagnosticsView.cpp =====
#include "DiagnosticsView.hpp"
#include "../widgets/StatusPanelWidget.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QFont>

DiagnosticsView::DiagnosticsView(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);

    auto* title = new QLabel("Диагностика", this);
    title->setFont(QFont("Segoe UI", 14, QFont::Bold));
    root->addWidget(title);

    statusPanel_ = new StatusPanelWidget(this);
    root->addWidget(statusPanel_);

    auto* logFrame = new QScrollArea(this);
    logLabel_ = new QLabel("", this);
    logLabel_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    logLabel_->setWordWrap(true);
    logLabel_->setFont(QFont("Courier New", 9));
    logLabel_->setStyleSheet(
        "color: #223027; background: #FFFFFF; "
        "border: 1px solid #D8E1D9; border-radius: 4px; padding: 8px;");
    logFrame->setWidget(logLabel_);
    logFrame->setWidgetResizable(true);
    root->addWidget(logFrame, 1);

    auto* btnRow = new QHBoxLayout();
    clearBtn_ = new QPushButton("Очистить лог", this);
    btnRow->addStretch();
    btnRow->addWidget(clearBtn_);
    root->addLayout(btnRow);

    connect(clearBtn_, &QPushButton::clicked, this, [this]() {
        logText_.clear();
        logLabel_->clear();
    });
}

void DiagnosticsView::setUartStatus(const QString& v)    { statusPanel_->setUartStatus(v);    }
void DiagnosticsView::setScannerStatus(const QString& v) { statusPanel_->setScannerStatus(v); }
void DiagnosticsView::setRfidStatus(const QString& v)    { statusPanel_->setRfidStatus(v);    }
void DiagnosticsView::setModuleSerial(const QString& v)  { statusPanel_->setModuleSerial(v);  }
void DiagnosticsView::setBatteryLevel(int level)         { statusPanel_->setBatteryLevel(level); }

void DiagnosticsView::appendLog(const QString& line) {
    logText_ += line + "\n";
    logLabel_->setText(logText_);
}
