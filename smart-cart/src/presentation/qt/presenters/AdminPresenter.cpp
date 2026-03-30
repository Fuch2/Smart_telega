// ===== src/presentation/qt/presenters/AdminPresenter.cpp =====
#include "AdminPresenter.hpp"
#include "../viewmodels/AdminViewModel.hpp"

AdminPresenter::AdminPresenter(AdminViewModel* vm, QObject* parent)
    : QObject(parent)
    , vm_(vm)
{}

void AdminPresenter::onLoad() {
    if (vm_) vm_->load();
}
