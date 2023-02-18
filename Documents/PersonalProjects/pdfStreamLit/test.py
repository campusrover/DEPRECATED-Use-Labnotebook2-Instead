# from streamlit_extras.stoggle import stoggle
# from streamlit_toggle import st_toggle_switch
# import streamlit as st
# import pandas as pd
# from io import StringIO

# # stoggle(
# #     "Click me!",
# #     """🥷 Surprise! Here's some additional content""",
# # )


# # st_toggle_switch(
# #     label="Enable Setting?",
# #     key="switch_1",
# #     default_value=False,
# #     label_after=False,
# #     inactive_color="#D3D3D3",  # optional
# #     active_color="#11567f",  # optional
# #     track_color="#29B5E8",  # optional
# # )

# uploaded_file = st.file_uploader("Choose a file",type=['csv'])
# if uploaded_file is not None:
#     # To read file as bytes:
#     bytes_data = uploaded_file.getvalue()
#     # st.write(bytes_data)

#     # To convert to a string based IO:
#     stringio = StringIO(uploaded_file.getvalue().decode("utf-8"))
#     # st.write(stringio)

#     # To read file as string:
#     string_data = stringio.read()
#     # st.write(string_data)

#     # Can be used wherever a "file-like" object is accepted:
#     dataframe = pd.read_csv(uploaded_file)
#     st.write(dataframe.head(10))
#     # print(dataframe.head())

import streamlit as st
import pandas as pd
from pytz import country_names
from st_aggrid import AgGrid, GridUpdateMode, JsCode
from st_aggrid.grid_options_builder import GridOptionsBuilder


@st.experimental_memo
def load_data():
    df = pd.read_csv("CSV_samples/country-list.csv")
    return df


@st.experimental_memo
def convert_df(df):
    # IMPORTANT: Cache the conversion to prevent computation on every rerun
    return df.to_csv().encode("utf-8")


def execute_query(conn, df_sel_row, table_name):
    if not df_sel_row.empty:
        conn.cursor().execute(
            "CREATE OR REPLACE TABLE "
            f"{table_name}(COUNTRY string, CAPITAL string, TYPE string)"
        )
        write_pandas(
            conn=conn,
            df=df_sel_row,
            table_name=table_name,
            database="ONE",
            schema="PUBLIC",
            quote_identifiers=False,
        )


# Initialize connection.
# Uses st.experimental_singleton to only run once.
@st.experimental_singleton
def init_connection():
    return snowflake.connector.connect(**st.secrets["snowflake"])


# The code below is for the title and logo.
st.set_page_config(page_title="Dataframe with editable cells", page_icon="💾")
st.image(
    "https://emojipedia-us.s3.dualstack.us-west-1.amazonaws.com/thumbs/240/apple/325/floppy-disk_1f4be.png",
    width=100,
)
conn = init_connection()
df = load_data()
st.title("Dataframe with editable cells")
st.write("")
st.markdown(
    """This is a demo of a dataframe with editable cells, powered by 
[streamlit-aggrid](https://pypi.org/project/streamlit-aggrid/). 
You can edit the cells by clicking on them, and then export 
your selection to a `.csv` file (or send it to your Snowflake DB!)"""
)
st.write("")
st.write("")
st.subheader("① Edit and select cells")
st.info("💡 Hold the `Shift` (⇧) key to select multiple rows at once.")
st.caption("")
gd = GridOptionsBuilder.from_dataframe(df)
gd.configure_pagination(enabled=True)
gd.configure_default_column(editable=True, groupable=True)
gd.configure_selection(selection_mode="multiple", use_checkbox=True)
gridoptions = gd.build()
grid_table = AgGrid(
    df,
    gridOptions=gridoptions,
    update_mode=GridUpdateMode.SELECTION_CHANGED,
    theme="material",
)
sel_row = grid_table["selected_rows"]

st.subheader(" ② Check your selection")

st.write("")

df_sel_row = pd.DataFrame(sel_row)
csv = convert_df(df_sel_row)
if not df_sel_row.empty:
    st.write(df_sel_row)
st.download_button(
    label="Download to CSV",
    data=csv,
    file_name="results.csv",
    mime="text/csv",
)
st.write("")
st.write("")