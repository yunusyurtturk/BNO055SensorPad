#pragma once


namespace BNOControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Xml;

	public ref class UISettingsSaver
	{
	private:
		String ^pFileName;
	public:
		XmlDocument ^pXmlDoc;

		UISettingsSaver(String ^p_file_name) {
			pFileName = p_file_name;

			pXmlDoc = gcnew XmlDocument();
			pXmlDoc->Load(pFileName);

		}

		void SaveDirectionQuaternion(int id, float w, float x, float y, float z)
		{
			XmlNode ^p_chart_settings = pXmlDoc->SelectSingleNode("/Application/DirectionQuaternions");
			if (nullptr != p_chart_settings) {
				XmlElement ^element = (XmlElement ^)p_chart_settings;

				XmlNode ^p_source = element->SelectSingleNode(L"//Quaternion[@id='" + id + "']");


				if (nullptr == p_source) {

					XmlElement ^p_new_element = pXmlDoc->CreateElement("Quaternion");
					XmlAttribute ^id_attr = pXmlDoc->CreateAttribute("id");
						id_attr->Value = id.ToString();
					XmlAttribute ^w_attr = pXmlDoc->CreateAttribute("w");
						w_attr->Value = w.ToString();
					XmlAttribute ^x_attr = pXmlDoc->CreateAttribute("x");
						x_attr->Value = x.ToString();
					XmlAttribute ^y_attr = pXmlDoc->CreateAttribute("y");
						y_attr->Value = y.ToString();
					XmlAttribute ^z_attr = pXmlDoc->CreateAttribute("z");
						z_attr->Value = z.ToString();
					
						p_new_element->Attributes->Append(id_attr);
						p_new_element->Attributes->Append(w_attr);
						p_new_element->Attributes->Append(x_attr);
						p_new_element->Attributes->Append(y_attr);
						p_new_element->Attributes->Append(z_attr);
			
						element->AppendChild(p_new_element);

					pXmlDoc->Save(pFileName);

					
				}
				else {
					XmlElement ^element = (XmlElement ^)p_source;
					element->Attributes["w"]->Value = w.ToString();
					element->Attributes["x"]->Value = x.ToString();
					element->Attributes["y"]->Value = y.ToString();
					element->Attributes["z"]->Value = z.ToString();

					pXmlDoc->Save(pFileName);
				}


			}
		}
		void GetDirectionQuaternion(int id, float *w, float *x, float *y, float *z )
		{
			float w_read, x_read, y_read, z_read;
			float ret_val = 0;
			bool success = true;
			XmlNode ^p_chart_settings = pXmlDoc->SelectSingleNode("/Application/DirectionQuaternions");
			if (nullptr != p_chart_settings) {
				XmlElement ^element = (XmlElement ^)p_chart_settings;

				XmlNode ^p_source = element->SelectSingleNode(L"//Quaternion[@id='" + id + "']");
				if (nullptr != p_source) {

					XmlElement ^quaternion = (XmlElement ^)p_source;

					if (!Single::TryParse(quaternion->Attributes["w"]->Value, w_read)) {
						success = false;
					}
					if (!Single::TryParse(quaternion->Attributes["x"]->Value, x_read)) {
						success = false;
					}
					if (!Single::TryParse(quaternion->Attributes["y"]->Value, y_read)) {
						success = false;
					}
					if (!Single::TryParse(quaternion->Attributes["z"]->Value, z_read)) {
						success = false;
					}
				}
			}
			if (success) {
				*w = w_read;
				*x = x_read;
				*y = y_read;
				*z = z_read;
			}
			else {
				*w = 0.5;
				*x = 0.5;
				*y = 0.5;
				*z = 0.5;
			}



		}
		double GetChartAxisLimit(String ^attr_min_max)
		{
			double ret_val = 0;

			XmlNode ^p_chart_settings = pXmlDoc->SelectSingleNode("/Application/Chart");
			if (nullptr != p_chart_settings) {

				XmlElement ^element = (XmlElement ^)p_chart_settings;

				if (!Double::TryParse(element->Attributes[attr_min_max]->Value, ret_val)) {
					ret_val = 0;
				}
			}

			return ret_val;
		}
		void SaveChartAxis(String ^attr_min_max, float val)
		{
			XmlNode ^p_chart_settings = pXmlDoc->SelectSingleNode("/Application/Chart");
			if (nullptr != p_chart_settings) {

				XmlElement ^element = (XmlElement ^)p_chart_settings;
				element->Attributes[attr_min_max]->Value = val.ToString();
				pXmlDoc->Save(pFileName);
			}
		}
		void SaveDataSourceSelectedState(String ^data_source_name, bool is_checked)
		{
			XmlNode ^p_data_components = pXmlDoc->SelectSingleNode("/Application/SelectedDataComponents");
			if (nullptr != p_data_components) {

				XmlNode ^p_source = p_data_components->SelectSingleNode(L"//Source[@name='" + data_source_name + "']");
				if (nullptr != p_source) {

					XmlElement ^element = (XmlElement ^)p_source;
					element->Attributes["is_checked"]->Value = is_checked.ToString();

					pXmlDoc->Save(pFileName);
				}

			}
		}
		bool DataSourceSelected(String ^data_source_name)
		{
			bool ret_val = false;
			
			XmlNode ^p_data_components = pXmlDoc->SelectSingleNode("/Application/SelectedDataComponents");
			if (nullptr != p_data_components) {

				XmlNode ^p_source = p_data_components->SelectSingleNode(L"//Source[@name='"+ data_source_name+"']");
				if (nullptr != p_source) {

					XmlElement ^element = (XmlElement ^)p_source;

					Boolean is_node_checked = false;
					String ^str = element->Attributes["is_checked"]->Value;
					bool is_convertable = Boolean::TryParse(str, is_node_checked);
					if (is_convertable) {
						ret_val = is_node_checked;
					}
				}
				else {
					/* Add an unexisted node*/
					XmlElement ^p_new_element = pXmlDoc->CreateElement("Source");
					XmlAttribute ^p_name_attr = pXmlDoc->CreateAttribute("name");
						p_name_attr->Value = data_source_name;
					XmlAttribute ^p_is_checked_attr = pXmlDoc->CreateAttribute("is_checked");
					p_is_checked_attr->Value = "false";

					p_new_element->Attributes->Append(p_name_attr);
					p_new_element->Attributes->Append(p_is_checked_attr);
					p_data_components->AppendChild(p_new_element);

					pXmlDoc->Save(pFileName);

					/* Not exist, add it*/

				}
			}

			return ret_val;

		}





	};

};