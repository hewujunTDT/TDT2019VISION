<?xml version="1.0" encoding="utf-8"?>
<!-- ***************************************************************************
*  (c) 2004-2008 by Basler Vision Technologies
*  Section: Vision Components
*  Project: GenApi
*  Author:  Fritz Dierks
* $Header$
*
*  License: This file is published under the license of the EMVA GenICam  Standard Group. 
*  A text file describing the legal terms is included in  your installation as 'GenICam_license.pdf'. 
*  If for some reason you are missing  this file please contact the EMVA or visit the website
*  (http://www.genicam.org) for a full copy.
* 
*  THIS SOFTWARE IS PROVIDED BY THE EMVA GENICAM STANDARD GROUP "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE EMVA GENICAM STANDARD  GROUP 
*  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  SPECIAL, 
*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT  LIMITED TO, 
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  DATA, OR PROFITS; 
*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  THEORY OF LIABILITY, 
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) 
*  ARISING IN ANY WAY OUT OF THE USE  OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************** -->
<xsl:stylesheet version="1.0" xmlns:my_v1_0="http://www.genicam.org/GenApi/Version_1_0" xmlns:my_v1_1="http://www.genicam.org/GenApi/Version_1_1" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text" encoding="UTF-8" media-type="text/plain"/>
<xsl:include href="text.xsl" />
<xsl:template match="/">
//-----------------------------------------------------------------------------
//  (c) 2004-2008 by Basler Vision Technologies
//  Section: Vision Components
//  Project: GenApi
//-----------------------------------------------------------------------------
/*!
\file
*/
//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef <xsl:value-of select="/RegisterDescription/@VendorName"/>_<xsl:value-of select="/RegisterDescription/@ModelName"/>Impl_H
#define <xsl:value-of select="/RegisterDescription/@VendorName"/>_<xsl:value-of select="/RegisterDescription/@ModelName"/>Impl_H


#if defined(_MSC_VER)
#   pragma warning(push)
#   pragma warning(disable: 4068) // unknown pragma (generated by bullseye coverage pragmas)
#   if defined(NDEBUG)
        // You may get a C4702 unreachable code in optimized builds. Thats why we disable it only in release builds#   if defined(NDEBUG)
#       pragma warning(disable: 4702) // unreachable code
#   endif
#endif

#include &lt;GenApi/impl/GenApiImpl.h&gt;

#if defined(_MSC_VER) &amp;&amp; defined(NDEBUG)
#   pragma warning(pop)
#endif


//! The namespace containing the device's control interface and related enumeration types
namespace <xsl:value-of select="/RegisterDescription/@VendorName"/>_<xsl:value-of select="/RegisterDescription/@ModelName"/>
{

	//! <xsl:value-of select="/RegisterDescription/@ToolTip"/>
	class C<xsl:value-of select="/RegisterDescription/@ModelName"/>Impl : public GenApi::CNodeMap
	{
	public:
		//! Constructor
		C<xsl:value-of select="/RegisterDescription/@ModelName"/>Impl(const GenICam::gcstring&amp; DeviceName);

		//! creates a nodemap object
		static INodeMap* Create(const GenICam::gcstring&amp; DeviceName);

		//! Makes the object to destroy itself
		virtual void Destroy(void);

		//----------------------------------------------------------------------------------------------------------------
		// Features
		//----------------------------------------------------------------------------------------------------------------
		<xsl:apply-templates select="/RegisterDescription/*" mode="DeclaringMembers"/>

	private:
		//! Function doing final initializations
		void FinalConstruct(void);

		//----------------------------------------------------------------------------------------------------------------
		// Initialization methods
		//----------------------------------------------------------------------------------------------------------------
		<xsl:apply-templates select="/RegisterDescription/*" mode="InitializingMethodDefinition"/>
	};
}

#endif // <xsl:value-of select="/RegisterDescription/@VendorName"/>_<xsl:value-of select="/RegisterDescription/@ModelName"/>Impl_H
</xsl:template>

<!-- =========================================================================== -->
<!-- DeclaringMembers                                                                                                                                -->
<!-- =========================================================================== -->

	<xsl:template match="*" mode="DeclaringMembers">
		/*! \brief <xsl:value-of select="./ToolTip"/><xsl:text>&#10;</xsl:text>
		
        <xsl:call-template name="line1" />
         <xsl:call-template name="plainstr" >
 			<xsl:with-param name="substr" select="./Description" />
			<xsl:with-param name="ncol" select="80" />
         </xsl:call-template>
         <xsl:call-template name="line1" />
         <xsl:text>*/</xsl:text>
		<xsl:text>		GenApi::</xsl:text><xsl:variable name="NodeType" select="name()"/>
		<xsl:value-of select="document('NodeTypes.xml')/NodeTypes/Node/CppClass[../@Name=$NodeType]"/><xsl:text> </xsl:text>
		<xsl:value-of select="@Name"/>;
	</xsl:template>
	<xsl:template match="Enumeration" mode="DeclaringMembers">
		/*! <xsl:value-of select="./ToolTip"/><xsl:text>&#10;</xsl:text>
			<xsl:text>&#10;</xsl:text>
		    <xsl:value-of select="./Description"/><xsl:text>&#10;</xsl:text>
		<xsl:text> */ &#10;</xsl:text>
		<xsl:text>		GenApi::</xsl:text><xsl:variable name="NodeType" select="name()"/>
		<xsl:value-of select="document('NodeTypes.xml')/NodeTypes/Node/CppClass[../@Name=$NodeType]"/><xsl:text> </xsl:text><xsl:value-of select="@Name"/>;
	</xsl:template>

<!-- =========================================================================== -->
<!-- InitializingMethodDefinition                                                                                                                                -->
<!-- =========================================================================== -->

	<xsl:template match="*" mode="InitializingMethodDefinition">
		void SetProperties<xsl:value-of select="@Name"/>();
	</xsl:template>

</xsl:stylesheet>
