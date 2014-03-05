#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=empl/inv_mpu.c empl/inv_mpu_dmp_motion_driver.c main.c device_i2c.c console.c time.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/empl/inv_mpu.o ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o ${OBJECTDIR}/main.o ${OBJECTDIR}/device_i2c.o ${OBJECTDIR}/console.o ${OBJECTDIR}/time.o
POSSIBLE_DEPFILES=${OBJECTDIR}/empl/inv_mpu.o.d ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/device_i2c.o.d ${OBJECTDIR}/console.o.d ${OBJECTDIR}/time.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/empl/inv_mpu.o ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o ${OBJECTDIR}/main.o ${OBJECTDIR}/device_i2c.o ${OBJECTDIR}/console.o ${OBJECTDIR}/time.o

# Source Files
SOURCEFILES=empl/inv_mpu.c empl/inv_mpu_dmp_motion_driver.c main.c device_i2c.c console.c time.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX250F128D
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/empl/inv_mpu.o: empl/inv_mpu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/empl 
	@${RM} ${OBJECTDIR}/empl/inv_mpu.o.d 
	@${RM} ${OBJECTDIR}/empl/inv_mpu.o 
	@${FIXDEPS} "${OBJECTDIR}/empl/inv_mpu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/empl/inv_mpu.o.d" -o ${OBJECTDIR}/empl/inv_mpu.o empl/inv_mpu.c   
	
${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o: empl/inv_mpu_dmp_motion_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/empl 
	@${RM} ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d 
	@${RM} ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o 
	@${FIXDEPS} "${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d" -o ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o empl/inv_mpu_dmp_motion_driver.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/device_i2c.o: device_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/device_i2c.o.d 
	@${RM} ${OBJECTDIR}/device_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/device_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/device_i2c.o.d" -o ${OBJECTDIR}/device_i2c.o device_i2c.c   
	
${OBJECTDIR}/console.o: console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/console.o.d 
	@${RM} ${OBJECTDIR}/console.o 
	@${FIXDEPS} "${OBJECTDIR}/console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/console.o.d" -o ${OBJECTDIR}/console.o console.c   
	
${OBJECTDIR}/time.o: time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/time.o.d 
	@${RM} ${OBJECTDIR}/time.o 
	@${FIXDEPS} "${OBJECTDIR}/time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/time.o.d" -o ${OBJECTDIR}/time.o time.c   
	
else
${OBJECTDIR}/empl/inv_mpu.o: empl/inv_mpu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/empl 
	@${RM} ${OBJECTDIR}/empl/inv_mpu.o.d 
	@${RM} ${OBJECTDIR}/empl/inv_mpu.o 
	@${FIXDEPS} "${OBJECTDIR}/empl/inv_mpu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/empl/inv_mpu.o.d" -o ${OBJECTDIR}/empl/inv_mpu.o empl/inv_mpu.c   
	
${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o: empl/inv_mpu_dmp_motion_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/empl 
	@${RM} ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d 
	@${RM} ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o 
	@${FIXDEPS} "${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o.d" -o ${OBJECTDIR}/empl/inv_mpu_dmp_motion_driver.o empl/inv_mpu_dmp_motion_driver.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/device_i2c.o: device_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/device_i2c.o.d 
	@${RM} ${OBJECTDIR}/device_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/device_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/device_i2c.o.d" -o ${OBJECTDIR}/device_i2c.o device_i2c.c   
	
${OBJECTDIR}/console.o: console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/console.o.d 
	@${RM} ${OBJECTDIR}/console.o 
	@${FIXDEPS} "${OBJECTDIR}/console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/console.o.d" -o ${OBJECTDIR}/console.o console.c   
	
${OBJECTDIR}/time.o: time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/time.o.d 
	@${RM} ${OBJECTDIR}/time.o 
	@${FIXDEPS} "${OBJECTDIR}/time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -DFOOTSENSE_TARGET_PIC32 -DMPU9150 -I"empl" -MMD -MF "${OBJECTDIR}/time.o.d" -o ${OBJECTDIR}/time.o time.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/pic32-mpu9150.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
