/****************************************************************************
** Meta object code from reading C++ file 'AdminViewModel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/presentation/qt/viewmodels/AdminViewModel.hpp"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'AdminViewModel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_AdminViewModel_t {
    uint offsetsAndSizes[30];
    char stringdata0[15];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[14];
    char stringdata4[8];
    char stringdata5[13];
    char stringdata6[9];
    char stringdata7[10];
    char stringdata8[7];
    char stringdata9[10];
    char stringdata10[9];
    char stringdata11[7];
    char stringdata12[13];
    char stringdata13[3];
    char stringdata14[13];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_AdminViewModel_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_AdminViewModel_t qt_meta_stringdata_AdminViewModel = {
    {
        QT_MOC_LITERAL(0, 14),  // "AdminViewModel"
        QT_MOC_LITERAL(15, 12),  // "modulesReset"
        QT_MOC_LITERAL(28, 0),  // ""
        QT_MOC_LITERAL(29, 13),  // "errorOccurred"
        QT_MOC_LITERAL(43, 7),  // "message"
        QT_MOC_LITERAL(51, 12),  // "infoOccurred"
        QT_MOC_LITERAL(64, 8),  // "loadDemo"
        QT_MOC_LITERAL(73, 9),  // "addModule"
        QT_MOC_LITERAL(83, 6),  // "serial"
        QT_MOC_LITERAL(90, 9),  // "slotCount"
        QT_MOC_LITERAL(100, 8),  // "firmware"
        QT_MOC_LITERAL(109, 6),  // "status"
        QT_MOC_LITERAL(116, 12),  // "updateModule"
        QT_MOC_LITERAL(129, 2),  // "id"
        QT_MOC_LITERAL(132, 12)   // "removeModule"
    },
    "AdminViewModel",
    "modulesReset",
    "",
    "errorOccurred",
    "message",
    "infoOccurred",
    "loadDemo",
    "addModule",
    "serial",
    "slotCount",
    "firmware",
    "status",
    "updateModule",
    "id",
    "removeModule"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_AdminViewModel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   56,    2, 0x06,    1 /* Public */,
       3,    1,   57,    2, 0x06,    2 /* Public */,
       5,    1,   60,    2, 0x06,    4 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       6,    0,   63,    2, 0x0a,    6 /* Public */,
       7,    4,   64,    2, 0x0a,    7 /* Public */,
      12,    5,   73,    2, 0x0a,   12 /* Public */,
      14,    1,   84,    2, 0x0a,   18 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::QString,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Int, QMetaType::QString, QMetaType::QString,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, QMetaType::Int, QMetaType::QString, QMetaType::QString,   13,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::Int,   13,

       0        // eod
};

Q_CONSTINIT const QMetaObject AdminViewModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_AdminViewModel.offsetsAndSizes,
    qt_meta_data_AdminViewModel,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_AdminViewModel_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<AdminViewModel, std::true_type>,
        // method 'modulesReset'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'errorOccurred'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'infoOccurred'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'loadDemo'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'addModule'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'updateModule'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'removeModule'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void AdminViewModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AdminViewModel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->modulesReset(); break;
        case 1: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->infoOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->loadDemo(); break;
        case 4: _t->addModule((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[4]))); break;
        case 5: _t->updateModule((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[5]))); break;
        case 6: _t->removeModule((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AdminViewModel::*)();
            if (_t _q_method = &AdminViewModel::modulesReset; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AdminViewModel::*)(const QString & );
            if (_t _q_method = &AdminViewModel::errorOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AdminViewModel::*)(const QString & );
            if (_t _q_method = &AdminViewModel::infoOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject *AdminViewModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AdminViewModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AdminViewModel.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int AdminViewModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void AdminViewModel::modulesReset()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void AdminViewModel::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AdminViewModel::infoOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
