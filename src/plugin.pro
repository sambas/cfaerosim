!win32 {
    error("AeroSimRC plugin is only available for win32 platform")
}

QT += serialport
QT -= gui

TEMPLATE = lib
TARGET = plugin_AeroSIMRC

copydata = 1

RES_DIR    = $${PWD}/resources
SIM_DIR    = "C:/CF_HIL/AeroSIM-RC"
PLUGIN_DIR = "C:/CF_HIL/AeroSIM-RC/Plugin/CleanFlight"
DLLDESTDIR = $$PLUGIN_DIR

defineReplace(targetPath) {
    return($$replace(1, /, $$QMAKE_DIR_SEP))
}

defineReplace(addNewline) {
    return($$escape_expand(\\n\\t))
}

HEADERS = \
    aerosimrcdatastruct.h \
    enums.h \
    plugin.h \
    qdebughandler.h \
    settings.h \
    fc_serial.h

SOURCES = \
    qdebughandler.cpp \
    plugin.cpp \
    settings.cpp \
    fc_serial.cpp

# Resemble the AeroSimRC directory structure and copy plugin files and resources
equals(copydata, 1) {

    # Windows release only
    win32:CONFIG(release, debug|release) {

        #data_copy.commands += -@$(MKDIR) $$targetPath(\"$$PLUGIN_DIR\") $$addNewline()

        # resources and sample configuration
        PLUGIN_RESOURCES = \
                cc_off.tga \
                cc_off_hover.tga \
                cc_on.tga \
                cc_on_hover.tga \
                cc_plugin.ini \
                plugin.txt
        for(res, PLUGIN_RESOURCES) {
            data_copy.commands += $(COPY_FILE) $$targetPath(\"$$RES_DIR/$$res\") $$targetPath(\"$$PLUGIN_DIR/$$res\") $$addNewline()
        }

        # Qt DLLs
        QT_DLLS = \
                  Qt5Gui.dll \
                  Qt5Core.dll \
                  Qt5SerialPort.dll \
                  Qt5Widgets.dll
        for(dll, QT_DLLS) {
            data_copy.commands += $(COPY_FILE) $$targetPath(\"$$[QT_INSTALL_BINS]/$$dll\") $$targetPath(\"$$SIM_DIR/$$dll\") $$addNewline()
        }

        # MinGW DLLs
        MINGW_DLLS = \
                     libwinpthread-1.dll \
                     libgcc_s_dw2-1.dll \
                     libstdc++-6.dll
        for(dll, MINGW_DLLS) {
#            data_copy.commands += $(COPY_FILE) $$targetPath(\"$$(QTMINGW)/$$dll\") $$targetPath(\"$$SIM_DIR/$$dll\") $$addNewline()
            data_copy.commands += $(COPY_FILE) $$targetPath(\"$$[QT_INSTALL_BINS]/$$dll\") $$targetPath(\"$$SIM_DIR/$$dll\") $$addNewline()
        }

        data_copy.target = FORCE
        QMAKE_EXTRA_TARGETS += data_copy
    }
}

include(..\qt-solutions\qtwinmigrate\src\qtwinmigrate.pri)
