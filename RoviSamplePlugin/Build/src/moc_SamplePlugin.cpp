/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    QByteArrayData data[25];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SamplePlugin"
QT_MOC_LITERAL(1, 13, 10), // "btnPressed"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 5), // "timer"
QT_MOC_LITERAL(4, 31, 8), // "getImage"
QT_MOC_LITERAL(5, 40, 11), // "get25DImage"
QT_MOC_LITERAL(6, 52, 20), // "stateChangedListener"
QT_MOC_LITERAL(7, 73, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(8, 95, 5), // "state"
QT_MOC_LITERAL(9, 101, 15), // "checkCollisions"
QT_MOC_LITERAL(10, 117, 11), // "Device::Ptr"
QT_MOC_LITERAL(11, 129, 6), // "device"
QT_MOC_LITERAL(12, 136, 5), // "State"
QT_MOC_LITERAL(13, 142, 17), // "CollisionDetector"
QT_MOC_LITERAL(14, 160, 8), // "detector"
QT_MOC_LITERAL(15, 169, 1), // "Q"
QT_MOC_LITERAL(16, 171, 1), // "q"
QT_MOC_LITERAL(17, 173, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(18, 194, 4), // "from"
QT_MOC_LITERAL(19, 199, 2), // "to"
QT_MOC_LITERAL(20, 202, 6), // "extend"
QT_MOC_LITERAL(21, 209, 7), // "maxTime"
QT_MOC_LITERAL(22, 217, 21), // "printProjectionMatrix"
QT_MOC_LITERAL(23, 239, 11), // "std::string"
QT_MOC_LITERAL(24, 251, 9) // "frameName"

    },
    "SamplePlugin\0btnPressed\0\0timer\0getImage\0"
    "get25DImage\0stateChangedListener\0"
    "rw::kinematics::State\0state\0checkCollisions\0"
    "Device::Ptr\0device\0State\0CollisionDetector\0"
    "detector\0Q\0q\0createPathRRTConnect\0"
    "from\0to\0extend\0maxTime\0printProjectionMatrix\0"
    "std::string\0frameName"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    1,   58,    2, 0x08 /* Private */,
       9,    4,   61,    2, 0x08 /* Private */,
      17,    4,   70,    2, 0x08 /* Private */,
      22,    1,   79,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Bool, 0x80000000 | 10, 0x80000000 | 12, 0x80000000 | 13, 0x80000000 | 15,   11,    8,   14,   16,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 15, QMetaType::Double, QMetaType::Double,   18,   19,   20,   21,
    QMetaType::Void, 0x80000000 | 23,   24,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SamplePlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->getImage(); break;
        case 3: _t->get25DImage(); break;
        case 4: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 5: { bool _r = _t->checkCollisions((*reinterpret_cast< Device::Ptr(*)>(_a[1])),(*reinterpret_cast< const State(*)>(_a[2])),(*reinterpret_cast< const CollisionDetector(*)>(_a[3])),(*reinterpret_cast< const Q(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 6: _t->createPathRRTConnect((*reinterpret_cast< Q(*)>(_a[1])),(*reinterpret_cast< Q(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        case 7: _t->printProjectionMatrix((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SamplePlugin::staticMetaObject = { {
    &rws::RobWorkStudioPlugin::staticMetaObject,
    qt_meta_stringdata_SamplePlugin.data,
    qt_meta_data_SamplePlugin,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION
static constexpr unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', '!',
    // metadata version, Qt version, architectural requirements
    0, QT_VERSION_MAJOR, QT_VERSION_MINOR, qPluginArchRequirements(),
    0xbf, 
    // "IID"
    0x02,  0x78,  0x2a,  'd',  'k',  '.',  's',  'd', 
    'u',  '.',  'm',  'i',  'p',  '.',  'R',  'o', 
    'b',  'w',  'o',  'r',  'k',  '.',  'R',  'o', 
    'b',  'W',  'o',  'r',  'k',  'S',  't',  'u', 
    'd',  'i',  'o',  'P',  'l',  'u',  'g',  'i', 
    'n',  '/',  '0',  '.',  '1', 
    // "className"
    0x03,  0x6c,  'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n', 
    // "MetaData"
    0x04,  0xa3,  0x6c,  'd',  'e',  'p',  'e',  'n', 
    'd',  'e',  'n',  'c',  'i',  'e',  's',  0x80, 
    0x64,  'n',  'a',  'm',  'e',  0x6b,  'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x67,  'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x65,  '1',  '.',  '0',  '.',  '0', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
